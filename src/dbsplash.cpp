#include <cassert>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <queue>
#include "fg.h"
#include "mpi.h"
#include "common.h"
#include "mpi.h"
#include "timing.h"

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

inline Vec2<int> deflatten(int position, int width) {
  return {position / width, position % width};
}

class DistributedBeliefPropagator {
public:
  std::vector<std::shared_ptr<Variable>> pq;
  std::unordered_map<std::shared_ptr<Variable>, int> idx_map;
  int rank, n_procs;
  int send_msgs_every = 10;
  Token token;
  Token token_recv_buf;
  Token token_send_buf;
  const int over_partition_factor = 2;
  int n_iters = 0;
  float converge_threshold = 1e-5;
  MPI_Request token_recv_req;
  MPI_Request token_send_req;
  int send_msgs_cnt = 0;
  int recv_msgs_cnt = 0;
  Vec2<Vec2<float>> edge_potential;

  DistributedBeliefPropagator(std::shared_ptr<FactorGraph> fg, int bfs_depth) {
    this->fg = fg;
    this->bfs_depth = bfs_depth;
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

      std::shared_ptr<Variable> root = GetNextVariable();

      if (root != nullptr) {
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
      }

      // Receive external messages. 
      int pos = 0;
      for (int src = 0; src < n_procs; src++) {
        if (src == rank) continue; // Ignore itself.
        MPI_Status status;
        int flag;
        // Get message count before receiving them.
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

      // Promote updated variables except the root.
      for (auto it = var_updates.begin(); it != var_updates.end(); ++it) {
        if (it->first == root) continue;
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
      if (root != nullptr) {
        root->residual = 0.0;
        pq.push_back(root);
        idx_map[root] = ((int) pq.size()) - 1;
      }

      for (auto& msgs: boundary_msgs) {
        msgs.clear();
      }
      var_updates.clear();
    }
    /*
    std::vector<MPI_Request> reqs;
    while (!sent_reqs.empty()) {
      reqs.push_back(sent_reqs.front());
    }
    MPI_Waitall(reqs.size(), &reqs[0], MPI_STATUSES_IGNORE);
    */
  }

  bool TokenRing(std::vector<std::shared_ptr<Variable>>& pq) {
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
          // printf("Process %d send end signal to Process %d\n", rank, tgt);
          MPI_Send(&token, sizeof(Token), MPI_BYTE, tgt, TOKEN_T, 
                   MPI_COMM_WORLD);
        }
        return false;
      }
      MPI_Irecv(&token_recv_buf, sizeof(Token), MPI_BYTE, MPI_ANY_SOURCE,
                TOKEN_T, MPI_COMM_WORLD, &token_recv_req);
    }

    // If the process has not converged yet, continue iterations.
    if (!pq.empty() && pq[0]->residual > converge_threshold) {
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
    // If the token has passed 2 rounds, send END_SIGNAL to every one.
    // Calculate the token given the current token.
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
      if (n == nullptr) continue;
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

  std::vector<int> Merge() {
    std::vector<int> beliefs;
    for (std::shared_ptr<Variable> v: pq) {
      int belief = (v->belief.x > v->belief.y? -1 : 1) * 
        (flatten(v->position, fg->width) + 1);
      beliefs.push_back(belief);
    }
    int size = (int) beliefs.size();
    if (rank == 0) {
      int n_slaves = n_procs - 1;
      for (int i = 0; i < n_slaves; i++) {
        int count;
        MPI_Status status;
        MPI_Probe(MPI_ANY_SOURCE, DATA_T, MPI_COMM_WORLD, &status);
        MPI_Get_count(&status, MPI_INT, &count);
        beliefs.resize(size + count);
        MPI_Recv(&beliefs[size], count, MPI_INT, status.MPI_SOURCE, DATA_T, 
                 MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        size += count;
      }
    } else {
      MPI_Send(&beliefs[0], size, MPI_INT, /*target*/0, DATA_T, MPI_COMM_WORLD);
    }
    return beliefs;
  }

private:
  int bfs_depth;
  std::shared_ptr<FactorGraph> fg;
  std::vector<std::shared_ptr<Variable>> ConstructBFSOrdering(
      std::shared_ptr<Variable> v) {
    std::vector<std::shared_ptr<Variable>> ordered_variables;
    std::unordered_set<int> visited;
    std::queue<std::pair<std::shared_ptr<Variable>, int>> q;
    q.push(std::make_pair(v, 0));
    visited.insert(flatten(v->position, fg->width));
    while (!q.empty()) {
      auto& vh = q.front();
      auto v = vh.first;
      int h = vh.second;
      q.pop();
      ordered_variables.push_back(v);
      for (int i = 0; i < 4 && h < bfs_depth; i++) {
        std::shared_ptr<Variable> n = fg->GetNeighbor(v, i);
        if (n == nullptr) continue;
        int cord = flatten(n->position, fg->width);
        if (visited.find(cord) == visited.end()) {
          if (n->partition == rank && n->residual > converge_threshold) {
            q.push(std::make_pair(n, h + 1));
            visited.insert(flatten(n->position, fg->width));
          }
        }
      }
    }
    return ordered_variables;
  }

  std::shared_ptr<Variable> GetNextVariable() {
    if (pq.empty()) return nullptr;
    std::shared_ptr<Variable> root = pq[0];
    if (root->residual <= converge_threshold) return nullptr;
    std::shared_ptr<Variable> var = pq.back();
    pq[0] = var;
    pq.pop_back();
    idx_map.erase(root);
    idx_map[var] = 0;
    int cur_idx = 0;
    int heap_size = (int) pq.size();
    while (cur_idx < heap_size) {
      std::shared_ptr<Variable> largest = var;
      int largest_idx = cur_idx;
      int left_idx = (cur_idx << 1) + 1;
      int right_idx = (cur_idx << 1) + 2;
      if (left_idx < heap_size && (*pq[left_idx]) > (*largest)) {
        largest = pq[left_idx];
        largest_idx = left_idx;
      }
      if (right_idx < heap_size && (*pq[right_idx]) > (*largest)) {
        largest = pq[right_idx];
        largest_idx = right_idx;
      }
      if (largest_idx == cur_idx) break;
      std::iter_swap(pq.begin() + cur_idx, pq.begin() + largest_idx);
      idx_map[var] = idx_map[largest];
      idx_map[largest] = cur_idx;
      cur_idx = largest_idx;
    }
    return root;
  }

  void Promote(std::shared_ptr<Variable> var, float belief_change) {
    var->residual += belief_change;

    int idx = idx_map[var];
    while (idx != 0) {
      int p_idx = (idx - 1) >> 1;
      std::shared_ptr<Variable> parent = pq[p_idx];
      if ((*parent) >= (*var)) break;
      // Swap parent and var.
      std::iter_swap(pq.begin() + idx, pq.begin() + p_idx);
      idx_map[parent] = idx;
      idx_map[var] = p_idx;
      idx = p_idx;
    }
  }

  void Partition(std::shared_ptr<FactorGraph> fg) {
    // Assume that # processes is a power of 2.
    std::srand(15618);
    int targetlevel = 0;
    int _n_procs = n_procs;
    int power = 0;
    while (_n_procs >>= 1) {
      power++;
    }
    int div_c;
    int n_parts = n_procs * over_partition_factor;
    if (power % 2 == 0) {
      div_c = (int) sqrt(n_parts);
    }
    else {
      div_c = (int) sqrt(n_parts >> 1);
    }
    int div_r = n_parts / div_c;

    int blk_rs = updiv(fg->height, div_r);
    int blk_cs = updiv(fg->width, div_c);

    std::vector<int> part_id_to_process;
    for (int i = 0; i < n_parts; i++) {
      part_id_to_process.push_back(i % n_procs);
    } 
    // For over-partitioning.
    std::random_shuffle(part_id_to_process.begin(), part_id_to_process.end());
    for (int i = 0; i < fg->height; i++) {
      for (int j = 0; j < fg->width; j++) {
        std::shared_ptr<Variable> var = fg->variables[i][j];
        int proc_id = part_id_to_process[i / blk_rs * div_c + j / blk_cs];
        var->partition = proc_id;
        if (proc_id == rank) {
          // Add variables to the priority queue.
          pq.push_back(var);
        }
      }
    }
    for (int idx = 0; idx < (int) pq.size(); idx++) {
      idx_map[pq[idx]] = idx;
    }
    printf("Process %d: I have %d variables\n", rank, (int) pq.size());
    printf("Process %d: I am responsible for partitions [", rank);
    for (int part_id = 0; part_id < (int) part_id_to_process.size(); part_id++) {
      if (part_id_to_process[part_id] == rank)
        printf("%d ", part_id);
    }
    printf("]\n");
  }
};

int main(int argc, char *argv[]) {
  printf("Program start.\n");
  MPI_Init(&argc, &argv);
  const char * filename = "data/noisy_fractal_triangles.txt";
  const char * output_path = "output/denoised_fractal_triangles.bmp";
  Image img = Image::ReadImage(filename);
  int rank;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  if (rank == 0) printf("Image size: %d X %d\n", img.h, img.w);
  std::shared_ptr<FactorGraph> fg = std::make_shared<FactorGraph>(img.pixels);
  DistributedBeliefPropagator dbp(fg, 2);

  Timer t;
  t.reset();
  dbp.BeliefPropagate();
  double elapsed = t.elapsed();
  printf("Process %d Converged in %.6fms\n", rank, elapsed);
  printf("Process %d Converged after %d iterations!\n", rank, dbp.n_iters);

  std::vector<int> beliefs = dbp.Merge();
  printf("Process %d: n_iters: %d\n", rank, dbp.n_iters);
  fg->variables.clear();
  if (rank == 0) {
    for (int b: beliefs) {
      int pred = b < 0? 0: 1;
      Vec2<int> position = deflatten(std::abs(b) - 1, fg->width);
      img.pixels[position.x][position.y] = pred;
    }
    img.SaveToFile(output_path);
  }
  MPI_Finalize();
}
