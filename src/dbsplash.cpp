#include <cmath>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "fg.h"
#include "mpi.h"
#include "common.h"
#include "mpi.h"

#define MSG_T 0
#define TOKEN_T 1
#define DATA_T 2
#define SIZE_T 3

const int END_SIGNAL = -2;
const int EMPTY_TOKEN = -1;

inline int updiv(int x, int y) {
  return (x + y - 1) / y;
}

struct deref_cmp {
  bool operator() (const std::shared_ptr<Variable> v1, 
                   const std::shared_ptr<Variable> v2) {
    return *v1 < *v2;
  }
};

struct Token {
  int m = 0;
  int recv_msgs_cnt = 0;
  int send_msgs_cnt = 0;
};


inline int opposite(int direction) {
  return (direction + 2) % 4;
}

inline int flatten(Vec2<int>& position, int width) {
  return position.x * width + position.y;
}

class DistributedBeliefPropagator {
public:
  std::vector<std::shared_ptr<Variable>> pq;
  std::unordered_map<std::shared_ptr<Variable>, int> idx_map;
  int rank, n_procs;
  int send_msgs_every = 1;
  Token token;
  Token token_recv_buf;
  Token token_send_buf;
  int n_iters = 0;
  float converge_threshold = 1e-5;
  MPI_Request token_recv_req;
  MPI_Request token_send_req;
  int send_msgs_cnt = 0;
  int recv_msgs_cnt = 0;
  Vec2<Vec2<float>> edge_potential;

  DistributedBeliefPropagator(std::shared_ptr<FactorGraph> fg, int bfs_depth) 
    : fg(fg) {
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &n_procs);
    if (rank == 0) token.m = 0;
    else token.m = -1;
    edge_potential.x = Vec2<float>(1.0, PHI);
    edge_potential.y = Vec2<float>(PHI, 1.0);
    Partition(fg);
  }

  void BeliefPropagate() {
    std::vector<std::vector<Message>> boundary_msgs(n_procs);
    std::vector<Message> received_msgs;
    std::unordered_map<std::shared_ptr<Variable>, float> var_updates;
    std::vector<Message> msgs;
    std::queue<MPI_Request> sent_reqs;
    std::queue<Message*> msg_buf;
    n_iters = 0;

    // Subscribe the token ring.
    MPI_Irecv(&token_recv_buf, sizeof(Token), MPI_BYTE, MPI_ANY_SOURCE, 
              TOKEN_T, MPI_COMM_WORLD, &token_recv_req);
    while (TokenRing(pq)) {

      std::shared_ptr<Variable> root = pq[0];
      if (root->residual > converge_threshold) {

        // Grow a spanning tree.
        const std::vector<std::shared_ptr<Variable>>& ordered_variables = 
          ConstructBFSOrdering(root);

        // From leaves to root.
        for (auto it = ordered_variables.rbegin();
             it != ordered_variables.rend(); ++it) {
          SendMessages(*it, boundary_msgs, var_updates);
        }
        // From root to leaves.
        for (auto& var: ordered_variables) {
          SendMessages(var, boundary_msgs, var_updates);
        }
      }

      // Receive external messages. 
      int pos = 0;
      for (int src = 0; src < n_procs; src++) {
        if (src == rank) continue; // Ignore itself.
        MPI_Status status;
        int flag;
        // Get # messages before receiving them.
        MPI_Iprobe(src, MSG_T, MPI_COMM_WORLD, &flag, &status);
        if (flag) {
          int count;
          MPI_Get_count(&status, MPI_BYTE, &count);
          int num_msgs = count / sizeof(Message);
          received_msgs.reserve((int) pos + num_msgs);
          MPI_Recv(&received_msgs[pos], count, MPI_BYTE, src, MSG_T, 
                   MPI_COMM_WORLD, MPI_STATUS_IGNORE);
          pos += num_msgs;
          recv_msgs_cnt += num_msgs;
        }
      }

      // Update with all received boundary messages.
      for (Message msg: received_msgs) {
        int x = msg.position.x, y = msg.position.y;
        std::shared_ptr<Variable> var = fg->variables[x][y];
        float belief_change = var->ReceiveMessage(msg.message, msg.direction);
        var_updates[var] += belief_change;
      }
      received_msgs.clear();

      // Promote updated variables.
      for (auto it = var_updates.begin(); it != var_updates.end(); ++it) {
        Promote(it->first, it->second);
      }

      n_iters += 1;

      // Send external messages every x iterations.
      if (n_iters % send_msgs_every == 0) {
        for (int tgt = 0; tgt < n_procs; tgt++) {
          if (tgt == rank) continue; // Ignore itself.
          msgs = boundary_msgs[tgt];
          if (!msgs.empty()) {
            MPI_Request req;
            int count = (int) msgs.size() * sizeof(Message);
            MPI_Isend(&msgs[0], count, MPI_BYTE, tgt, MSG_T, MPI_COMM_WORLD, 
                      &req);
            send_msgs_cnt += (int) msgs.size();
            sent_reqs.push(req);
            boundary_msgs[tgt] = std::vector<Message>();
          }
        }
      }
      int flag = !sent_reqs.empty();
      while (flag) {
        MPI_Request sent_req = sent_reqs.front();
        MPI_Test(&sent_req, &flag, MPI_STATUS_IGNORE);
        if (flag) sent_reqs.pop();
        flag = (!sent_reqs.empty()) && flag;
      }

      // Append root to the end of the priority queue.
      std::pop_heap(pq.begin(), pq.end());
      root->residual = 0.0;

      for (auto& msgs: boundary_msgs) {
        msgs.clear();
      }
      var_updates.clear();
    }
    printf("Waiting for all remaining sent_reqs.\n");
    std::vector<MPI_Request> reqs;
    while (!sent_reqs.empty()) {
      reqs.push_back(sent_reqs.front());
    }
    MPI_Waitall(reqs.size(), &reqs[0], MPI_STATUSES_IGNORE);
    printf("Process %d finished.\n", rank);
  }

  bool TokenRing(std::vector<std::shared_ptr<Variable>>& pq) {
    // If the token has passed 2 rounds, send END_SIGNAL to every one.
    // Calculate the token given the current token.
    /*
    if (token >= 0) {
      if (pq.top().residual > converge_threshold) {
        // Reset the token if the processor holds it has not converged.
        if (token > 0) token = 0;
      } else {
        token += 1;
      }
      MPI_Irecv(&token_recv_buf, 1, sizeof(MPI_INT), MPI_ANY_SOURCE,
                TOKEN_T, &token_recv_req, MPI_COMM_WORLD);
      MPI_Send(&token, 1, MPI_INT, (rank + 1) % n_procs, 1, 
               sizeof(MPI_INT), MPI_COMM_WORLD);
      MPI_Wait(&token_recv_req, MPI_STATUS_IGNORE);
      token = token_recv_buf;
      if (token == n_procs * 2) {
        
      }
    }
    */

    int flag;
    // Check for token receipt.
    MPI_Test(&token_recv_req, &flag, MPI_STATUS_IGNORE);
    if (flag) {
      token = token_recv_buf;
      if (token.m == END_SIGNAL) {
        return false;
      }
      if (token.m >= 2 * n_procs && 
          token.send_msgs_cnt == token.recv_msgs_cnt) {
        // Everyone has converged, send end signal to every one.
        token.m = END_SIGNAL;
        for (int tgt = 0; tgt < n_procs; tgt++) {
          if (tgt == rank) continue;
          printf("Process %d send end signal to Process %d\n", rank, (rank + 1) % n_procs);
          MPI_Send(&token, sizeof(Token), MPI_BYTE, (rank + 1) % n_procs, 
                   TOKEN_T, MPI_COMM_WORLD);
        }
        return false;
      }
      MPI_Irecv(&token_recv_buf, sizeof(Token), MPI_BYTE, MPI_ANY_SOURCE,
                TOKEN_T, MPI_COMM_WORLD, &token_recv_req);
    }

    // If the process has not converged yet, continue iterations.
    if (pq[0]->residual > converge_threshold) {
      // Reset the token if the processor holds it.
      if (token.m > 0) token.m = 0;
      return true;
    }

    // Otherwise, the process has temperarily converged.
    // If it has the token, send the token to the next processor.
    if (token.m >= 0) {
      token.m += 1;
      token.send_msgs_cnt += send_msgs_cnt;
      token.recv_msgs_cnt += recv_msgs_cnt;
      send_msgs_cnt = 0;
      recv_msgs_cnt = 0;
      // Wait until the token is received by the next processor.
      MPI_Send(&token, sizeof(Token), MPI_BYTE, (rank + 1) % n_procs, 
               TOKEN_T, MPI_COMM_WORLD);
      token.m = -1;
    }
    return true;
  }

  void SendMessages(
    std::shared_ptr<Variable> v, 
    std::vector<std::vector<Message>>& boundary_msgs,
    std::unordered_map<std::shared_ptr<Variable>, float>& var_updates
  ) {
    // belief.
    Vec2<float> belief = v->belief;

    // Update in-messages for each of the variable's neighbors.
    for (int i = 0; i < 4; i++) {
      Vec2<float> out_msg = belief / v->in_msgs[i];
      Vec2<float> recv_msg = Vec2<float>(
        out_msg.x + out_msg.y * PHI, 
        out_msg.x * PHI + out_msg.y
      ).normalize();
      int j = opposite(i);
      std::shared_ptr<Variable> n = fg->GetNeighbor(v, i);
      if (n->partition == rank) {
        // If the receiver is in the same partition,
        // directly update the in_msg.
        float belief_change = n->ReceiveMessage(recv_msg, j);
        var_updates[n] += belief_change;
      } else {
        // If the receiver is in another partition,
        // Store the message in boundary_msgs to send later.
        Message msg;
        msg.direction = j;
        msg.message = recv_msg;
        msg.position = n->position;
        boundary_msgs[n->partition].emplace_back(msg);
      }
    }
  }

  std::vector<Message> Merge() {
    std::vector<Message> beliefs;

    // collect beliefs of variables in current partition
    for (std::shared_ptr<Variable> v: pq) {
      Vec2<float> belief = v->belief;
      Message msg;
      msg.position = v->position;
      msg.message = belief;
      msg.direction = 0; // can be ignored
      beliefs.push_back(msg);
    }

    //start to merge with other processors;
    for (int i = 1; i < n_procs; i <<= 1) {
      int pair;
      if (rank % (i << 1) < i) {
          pair = rank + i;
      } else {
          pair = rank - i;
      }
      int curr_size = beliefs.size();
      MPI_Request req;
      int size;
      MPI_Irecv(&size, 1, MPI_INT, pair, SIZE_T, MPI_COMM_WORLD, &req);
      MPI_Send(&curr_size, 1, MPI_INT, pair, SIZE_T, MPI_COMM_WORLD);
      MPI_Wait(&req, MPI_STATUS_IGNORE);

      beliefs.resize(curr_size + size);

      //printf("beliefs size %d after resize %d\n", size, beliefs.size());
      MPI_Irecv(&beliefs[curr_size], sizeof(Message) * size, MPI_BYTE, pair, 
                DATA_T, MPI_COMM_WORLD, &req);
      MPI_Send(&beliefs[0], sizeof(Message) * curr_size, MPI_BYTE, pair, 
               DATA_T, MPI_COMM_WORLD);
      MPI_Wait(&req, MPI_STATUS_IGNORE);
      //printf("receiving normalized belief for v(%d, %d) is (%f, %f)\n", msg.position.x, msg.position.y,
      // msg.message.x, msg.message.y);
    }
    return beliefs;
    // first calculate beliefs
  }

private:
  int bfs_depth;
  const std::shared_ptr<FactorGraph> fg;
  std::vector<std::shared_ptr<Variable>> ConstructBFSOrdering(
      std::shared_ptr<Variable> v) {
    std::vector<std::shared_ptr<Variable>> ordered_variables;
    std::unordered_set<int> visited;
    std::queue<std::shared_ptr<Variable>> q;
    q.push(v);
    int h = bfs_depth;
    while (!q.empty() && h-- > 0) {
      std::shared_ptr<Variable> v = q.front();
      q.pop();
      ordered_variables.push_back(v);
      visited.insert(flatten(v->position, fg->width));
      for (int i = 0; i < 4; i++) {
        std::shared_ptr<Variable> n = fg->GetNeighbor(v, i);
        if (n == nullptr) continue;
        int cord = flatten(n->position, fg->width);
        if (visited.find(cord) != visited.end()) {
          if (n->partition == rank && n->residual > converge_threshold) {
            q.push(n);
          }
        }
      }
    }
    return ordered_variables;
  }

  void Promote(std::shared_ptr<Variable> var, float belief_change) {
    var->residual += belief_change;
    int idx = idx_map[var];
    while (idx != 0) {
      int p_idx = (idx - 1) >> 1;
      std::shared_ptr<Variable> parent = pq[p_idx];
      if (parent >= var) break;
      // Swap parent and var.
      std::iter_swap(pq.begin() + idx, pq.begin() + p_idx);
      int tmp = idx_map[parent];
      idx_map[parent] = idx_map[var];
      idx_map[var] = tmp;
      idx = p_idx;
    }
  }

  void Partition(std::shared_ptr<FactorGraph> fg) {
    // Assume that # processes is a power of 2.
    // std::srand(15618);
    int rank, n_procs;
    int targetlevel = 0;
    int _n_procs = n_procs;
    int power = 0;
    while (_n_procs >>= 1) {
      power++;
    }
    int div_c = (int) sqrt(n_procs);
    int div_r = div_c;
    if (power % 1) {
      div_r <<= 1;
    }
    int blk_rs = updiv(fg->height, div_r);
    int blk_cs = updiv(fg->width, div_c);

    int num_parts = n_procs;
    std::vector<int> part_id_to_process;
    for (int i = 0; i < num_parts; i++) {
      part_id_to_process.push_back(i % n_procs);
    }
    // std::random_shuffle(part_id_to_process); // For over-partitioning.
    for (int i = 0; i < fg->height; i++) {
      for (int j = 0; j < fg->width; j++) {
        std::shared_ptr<Variable> var = fg->variables[i][j];
        int proc_id = part_id_to_process[i / blk_rs * blk_cs + j / blk_cs];
        var->partition = proc_id;
        if (proc_id == rank) {
          // Add variables to the priority queue.
          pq.push_back(var);
        }
      }
    }
    std::make_heap(pq.begin(), pq.end(), deref_cmp());
    for (int idx = 0; idx < (int) pq.size(); idx++) {
      idx_map[pq[idx]] = idx;
    }
  }
};

int main(int argc, char *argv[]) {
  MPI_Init(&argc, &argv);
  Image img = Image::ReadImage("data/rice.txt");
  int rank;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  std::shared_ptr<FactorGraph> fg = std::make_shared<FactorGraph>(img.pixels);
  DistributedBeliefPropagator dbp(fg, 2);
  dbp.BeliefPropagate();
  std::vector<Message> beliefs = dbp.Merge();
  if (rank == 0) {
    FactorGraph::writeDenoisedImage(beliefs, "output/denoised_rice.txt");
    for (Message m : beliefs) {
      Vec2<float> norm_b = m.message.normalize();
      printf("normalized belief for v(%d, %d) is (%f, %f)\n", m.position.x, m.position.y,
      norm_b.x, norm_b.y);
    }
  }
  MPI_Finalize();
}
