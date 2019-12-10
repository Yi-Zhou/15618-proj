#include <cassert>
#include <unordered_set>
#include "common.h"
#include "fg.h"
#include "mpi.h"
#include "timing.h"

#define SIZE_TAG 1
#define DATA_TAG 2

class SynchronousBeliefPropagator
{
public:
  double beta;
  int rank, n_procs;
  Vec2<Vec2<float>> edge_potential;
  std::shared_ptr<FactorGraph> fg;
  std::unordered_set<int> neighbor_procs;

  SynchronousBeliefPropagator(std::shared_ptr<FactorGraph> fg)
  {
    beta = 1e-5;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &n_procs);
    edge_potential.x = Vec2<float>(1.0, 0.3);
    edge_potential.y = Vec2<float>(0.3, 1.0);
    this->fg = fg;
  }

  //Merge the results on different processors to each processor
  std::vector<Message> merge()
  {
    std::vector<Message> beliefs;

    // collect beliefs of variables in current partition
    for (auto vector : fg->variables)
    {
      for (std::shared_ptr<Variable> v : vector)
      {
        if (v->partition != rank)
        {
          continue;
        }
        Vec2<float> belief = v->calulateBelief();
        Message msg;
        msg.position = v->position;
        msg.message = belief;
        msg.direction = 0; // can be ignored
        Vec2<float> norm_b = v->belief;

        beliefs.push_back(msg);
      }
    }

    //start to merge with other processors;
    int tag1 = SIZE_TAG;
    int tag2 = DATA_TAG;
    for (int i = 1; i < n_procs; i <<= 1)
    {
      int pair;
      if (rank % (i << 1) < i)
      {
        pair = rank + i;
      }
      else
      {
        pair = rank - i;
      }
      int curr_size = beliefs.size();
      MPI_Request req;
      int size;
      MPI_Irecv(&size, 1, MPI_INT, pair, tag1, MPI_COMM_WORLD, &req);
      MPI_Send(&curr_size, 1, MPI_INT, pair, tag1, MPI_COMM_WORLD);
      MPI_Wait(&req, MPI_STATUS_IGNORE);

      beliefs.resize(curr_size + size);

      MPI_Irecv(&beliefs[curr_size], sizeof(Message) * size, MPI_BYTE,
                pair, tag2, MPI_COMM_WORLD, &req);
      MPI_Send(&beliefs[0], sizeof(Message) * curr_size, MPI_BYTE, pair, 
                tag2, MPI_COMM_WORLD);
      MPI_Wait(&req, MPI_STATUS_IGNORE);
    }

    assert(beliefs.size() == fg->width * fg->height);
    return beliefs;
  }

  // Update messages received from neighbors
  float updateInMessages(std::vector<Message> &in_messages)
  {
    float max_diff = 0;
    for (Message msg : in_messages)
    {
      Vec2<int> p = msg.position;
      std::shared_ptr<Variable> v = fg->variables[p.x][p.y];

      Vec2<float> old_message = v->in_msgs[msg.direction];
      Vec2<float> diff = old_message - msg.message;
      max_diff = std::max((old_message - msg.message).l1_norm(), max_diff);

      v->in_msgs[msg.direction] = msg.message;
    }
    return max_diff;
  }

  // Get the updated messages for neighbors
  void getOutMessages(std::vector<std::vector<Message>> &out_messages)
  {
    for (auto vector : fg->variables)
    {
      for (std::shared_ptr<Variable> v : vector)
      {

        if (v->partition != rank)
        {
          continue;
        }

        Vec2<float> out_msg(1.0, 1.0);
        for (int i = 0; i < 5; i++)
        {
          out_msg *= v->in_msgs[i];
        }

        //calculate the out messages to neighbors using the equations
        for (int i = 0; i < 4; i++)
        {
          std::shared_ptr<Variable> neighbor = fg->GetNeighbor(v, i);
          if (neighbor != nullptr)
          {
            Vec2<float> prod = out_msg / v->in_msgs[i];
            int direction = (i + 2) % 4;
            Message msg;
            msg.direction = direction;
            msg.message = Vec2<float>(prod.x * edge_potential.x.x + 
                                      prod.y * edge_potential.y.x,
                                      prod.x * edge_potential.x.y + 
                                      prod.y * edge_potential.y.y).normalize();

            msg.position = Vec2<int>(neighbor->position.x, 
                                      neighbor->position.y);
            out_messages[neighbor->partition].push_back(msg);
          }
        }
      }
    }
  }

  // Synchronous belief propagate. Synchronize at the end of each iteration
  bool beliefPropagate()
  {
    std::unordered_set<int> neighbor_procs_set(neighbor_procs);

    int tag1 = SIZE_TAG; // transmit a size
    int tag2 = DATA_TAG; // transmit messages
    std::vector<MPI_Request> size_reqs;
    std::vector<MPI_Request> msg_reqs;

    std::vector<std::vector<Message>> out_messages(n_procs, 
                                                    std::vector<Message>());
    std::vector<Message> in_messages;

    int buffer[n_procs];

    // send messages to neighbors
    getOutMessages(out_messages);
    for (int i = 0; i < n_procs; i++)
    {
      if (!out_messages[i].empty())
      {
        if (i == rank)
        {
          // messages from the same partition
          in_messages.insert(in_messages.end(),
                             out_messages[i].begin(),
                             out_messages[i].end());
        }
        else
        {
          // messages from different partitions, i.e., processes
          int size = out_messages[i].size();
          MPI_Request size_req, msg_req;
          Message msg = out_messages[i][0];
          buffer[i] = size;
          MPI_Isend(&out_messages[i][0], sizeof(Message) * size, MPI_BYTE, 
                    i, tag2, MPI_COMM_WORLD, &msg_req);
          msg_reqs.push_back(msg_req);
          neighbor_procs_set.insert(i);
        }
      }
    }

    //receive the messages from neighbors
    while (!neighbor_procs_set.empty())
    {

      for (auto it = neighbor_procs_set.begin();
           it != neighbor_procs_set.end();)
      {

        int neighbor = *it;
        int flag;
        MPI_Status status;
        //probe to see whether there are incoming messages
        MPI_Iprobe(neighbor, tag2, MPI_COMM_WORLD, &flag, &status);
        if (flag)
        {
          int size;
          MPI_Get_count(&status, MPI_INT, &size);
          int curr_size = in_messages.size();
          in_messages.reserve(curr_size + size);
          MPI_Recv(&in_messages[curr_size], sizeof(Message) * size, MPI_BYTE,
                   neighbor, tag2, MPI_COMM_WORLD, &status);
          neighbor_procs_set.erase(it++);
        }
        else
        {
          ++it;
        }
      }
    }

    MPI_Status stats[msg_reqs.size()];
    MPI_Waitall(msg_reqs.size(), &msg_reqs[0], stats);

    float diff = updateInMessages(in_messages);
    return is_converged(diff);
  }

private:
  // check whether the algorithm has converged and this implementation 
  // is tested in toy_test
  bool is_converged(float diff)
  {
    MPI_Request req;
    int tag1 = 1;

    for (int i = 1; i < n_procs; i <<= 1)
    {
      int pair;
      if (rank % (i << 1) < i)
      {
        pair = rank + i;
      }
      else
      {
        pair = rank - i;
      }
      float other_max_diff;
      MPI_Irecv(&other_max_diff, 1, MPI_FLOAT, pair, tag1, 
                MPI_COMM_WORLD, &req);
      MPI_Send(&diff, 1, MPI_FLOAT, pair, tag1, MPI_COMM_WORLD);
      MPI_Wait(&req, MPI_STATUS_IGNORE);
      diff = std::max(other_max_diff, diff);
    }

    return diff < beta;
  }
};

int main(int argc, char *argv[])
{
  MPI_Init(&argc, &argv);
  if (argc != 5)
  {
    printf("Usage: mpirun -np [nproc] synchronous [max_iter] [data_file]" 
            "[partition_file] [out_file]\n");
    exit(1);
  }
  int max_iter = std::stoi(argv[1]);
  char *image_file = argv[2];
  char *partition_file = argv[3];
  char *out_file = argv[4];
  printf("run at most %d iterations\n", max_iter);
  // Use this small image to debug the correctness of this implementation
  // at very first
  // std::vector<std::vector<int>> img{{1, 1, 1, 1, 1, 1, 1, 1},
  //                                   {1, 1, 1, 1, 1, 1, 1, 1},
  //                                   {1, 0, 1, 1, 1, 0, 1, 1},
  //                                   {1, 1, 1, 1, 1, 1, 1, 1},
  //                                   {1, 1, 1, 0, 1, 1, 1, 1},
  //                                   {1, 1, 1, 1, 1, 1, 1, 1},
  //                                   {1, 1, 1, 1, 1, 0, 1, 1},
  //                                   {1, 1, 1, 1, 1, 1, 1, 1},
  //                                   };
  // Image& img = ReadImage(image);

  Image img = Image::ReadImage(image_file);

  double total_elapsed = 0.0;
  int num_iter = 5;
  for (int j = 0; j < num_iter; j++)
  {
    printf("Start %dth belief propagation!\n", j);
    std::shared_ptr<FactorGraph> fg = std::make_shared<FactorGraph>(img.pixels, 
                                                  partition_file);
    SynchronousBeliefPropagator bp(fg);

    int i = 0;

    Timer t;
    t.reset();

    while (!bp.beliefPropagate())
    {
      if (i > max_iter)
      {
        printf("Run %d iterations but still not converged!\n", max_iter);
        break;
      }
      i++;
    }

    double elapsed = t.elapsed();
    if (i < max_iter)
    {
      if (j > 0)
      {
        total_elapsed += elapsed;
      }
      printf("Converged in %.6fs after %d iterations!\n", elapsed, i);
    }
    else
    {
      printf("Fail to converged in %.6fs after %d iterations!\n", elapsed, i);
    }

    if (j == num_iter - 1)
    {
      std::vector<Message> beliefs = bp.merge();
      int rank;
      MPI_Comm_rank(MPI_COMM_WORLD, &rank);

      if (rank == 0)
      {
        assert(beliefs.size() == img.w * img.h);
        for (Message m : beliefs)
        {
          Vec2<float> norm_b = m.message.normalize();
          int color = norm_b.x > norm_b.y ? 0 : 1;
          img.pixels[m.position.x][m.position.y] = color;
        }
        img.SaveToFile(out_file);
      }
    }
  }
  printf("Averge convergence time %.6fs\n", total_elapsed / 4);

  MPI_Finalize();
}
