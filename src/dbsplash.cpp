#include <math.h>
#include <vector>
#include "fg.h"
#include "mpi.h"
#include "common.h"
#include "mpi.h"

#define DATA_TAG 0

inline int updiv(int x, int y) {
  return (x + y - 1) / y;
}

struct deref_cmp {
  bool operator() (const std::shared_ptr<Variable> v1, 
                   const std::shared_ptr<Variable> v2) {
    return *v1 < *v2;
  }
}

inline int opposite(int direction) {
  return (i + 2) % 4;
}

class DistributedBeliefPropagator {
public:
  std::priority_queue<std::shared_ptr<Variable>, 
                      std::vector<std::shared_ptr<Variable>,
                      deref_cmp> pq;
  std::vector<std::shared_ptr<Variable>> heap;
  std::unordered_map<std::shared_ptr<Variable>, int> idx_map;
  int rank, n_procs;
  int send_msgs_every = 10;

  BeliefPropagator(std::shared_ptr<FactorGraph> fg, int bfs_depth) : fg(fg) {
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &n_procs);
    edge_potential.x = Vec2<float>(1.0, PHI);
    edge_potential.y = Vec2<float>(PHI, 1.0);
    partition(fg);
  }

  void BeliefPropagate() {
    std::vector<Message> boundary_msgs;
    std::unordered_map<std::shared_ptr<Variable>> var_updates;
    std::vector<Message> msgs;
    int n_iters = 0;
    while (TokenRing(pq)) {
      std::shared_ptr<Variable> root = pq.top();

      // Grow a spanning tree.
      std::vector<std::shared_ptr<Variable>>& ordered_variables = 
        ConstructBFSOrdering(root);

      // From the leaves to the root.
      for (auto it = ordered_variables.rbegin();
           it != ordered_variables.rend(); ++it) {
        SendMessages(v, boundary_msgs, var_updates);
      }
      // From the root to the leaves.
      for (auto& var: ordered_variables) {
        SendMessages(var, boundary_msgs, var_updates);
      }

      // Try to receive external messages.
      for (int src = 0; src < n_procs, p++) {
        if (src == rank) continue; // Ignore itself.
        MPI_Status status;
        int flag;
        MPI_Iprobe(src, MSG_T, MPI_COMM_WORLD, &flag, &status);
        int count;
        while (flag) {
          MPI_Get_count(&status, &count);
          msgs.resize(count/sizeof(Message))
          MPI_Recv(&msgs[0], MPI_BYTE, count);
          for (Message& msg: msgs) {
            int x = msg.position.x, y = msg.position.y;
            std::shared_ptr<Variable> var = fg->variables[x][y];
            float belief_change = var.ReceiveMessage(msg.message, 
                                                     msg.direction);
            var_updates[var] += belief_change;
          }
          MPI_Iprobe(src, MSG_T, MPI_COMM_WORLD, &flag, &status);
        }
      }

      // Promote updated variables.
      for (auto it = var_updates.begin(); var_updates.end(); ++it) {
        promote(it->first, it->second);
      }

      n_iters += 1;

      // Send external messages every 10 iterations.
      if (n_iters % send_msgs_every == 0) {
        for (int tgt = 0; tgt < n_procs; tgt++) {
          if (tgt == rank) continue;
          MPI_
          msg = msgs[rank];
        }
      }

      // Append root to the end of the priority queue.
      pq.pop();
      root.residual = 0.0;
      pq.push_back(root)
      std::push_heap(pq.begin(), pq.end());

      boundary_msgs.clear();
      updated_vars.clear();
    }
  }

  void SendMessages(
    std::shared_ptr<Variable> v, 
    std::vector<Message>& boundary_msgs,
    std::unordered_set<std::shared_ptr<Variable>>& updated_vars
  ) {
    // belief.
    Vec2<float> belief = v->belief;

    // Update in-messages for each of the variable's neighbors.
    for (int i = 0; i < 4; i++) {
      Vec2<float> out_msg = belief / v->in_msgs[i];
      recv_msg = Vec2<float>(
        out_msg.x + out_msg.y * PHI, 
        out_msg.x * PHI + out_msg.y
      ).normalize();
      int j = opposite(i);
      std::shared_ptr<Variable> n = fg->GetNeighbor(v, i);
      if (n->partition == rank) {
        // If the receiver is in the same partition,
        // directly update the in_msg.
        n->ReceiveMessage(recv_msg, j)
        updated_vars.insert(n);
      } else {
        // If the receiver is in another partition,
        // Store the message in boundary_msgs to send later.
        Message msg;
        msg.direction = j;
        msg.message = recv_msg;
        msg.position = n->position;
        boundary_msgs.emplace_back(msg);
      }
    }
  }
}

private:
  int bfs_depth;
  const std::shared_ptr<FactorGraph> fg;
  std::vector<std::shared_ptr<Variable>> ConstructBFSOrdering(
      std::shared_ptr<Variable> v) {
    std::vector<std::shared_ptr<Variable>> ordered_variables;
    std::unordered_map<std::pair<int, int>> visited;
    std::queue<std::shared_ptr<Variable>> q;
    q.push_back(v);
    int h = bfs_depth;
    while (!q.empty() && h-- > 0) {
      std::shared_ptr<Variable> v = q.pop_left();
      ordered_variables.pusb_back(v);
      visited.insert(std::make_pair<int, int>(v.i, v.j));
      for (std::shared_ptr<Varaible> n : v.neighbors) {
        auto& cord = std::make_pair<int, int>(n.i, n.j);
        if (visited.find(cord) != visited.end()) {
          if (var->process == rank) {
            q.push_back(n);
          }
        }
      }
    }
    return ordered_variables;
  }

  void promote(std::shared_ptr<Variable> var, float belief_change) {
    var.residual += belief_change;
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

  void partition(std::shared_ptr<FactorGraph> fg) {
    // Assume that # processes is a power of 2.
    // std::srand(15618);
    int rank, n_procs;
    int targetlevel = 0;
    int _n_procs = n_procs;
    power = 0;
    while (_n_procs >>= 1) {
      power++;
    }
    int div_c = (int) sqrt(n_procs);
    int div_r = div_c;
    if (power % 1) {
      div_r <<= 1;
    }
    blk_rs = updiv(fg->height, div_r);
    blk_cs = updiv(fg->width, div_c);

    int num_parts = n_procs;
    std::vector<int> part_id_to_process;
    for (int i = 0; i < num_parts; i++) {
      part_id_to_process.pusb_back(i % n_procs);
    }
    // std::random_shuffle(part_id_to_process); // For over-partitioning.
    for (int i = 0; i < fg->height; i++) {
      for (int j = 0; j < fg->width; j++) {
        std::shared_ptr<Variable> var = fg->variables[i][j];
        int proc_id = part_id_to_process[i / blk_rs * blk_cs + j / blk_cs];
        var->process = proc_id;
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

int main() {
  MPI_Init();
  Image& img = ReadImage(image);
  std::shared_ptr<FactorGraph> fg = std::make_shared<FactorGraph>(img);
  DistributedBeliefPropagator dbp(fg);
  MPI_Finalize();
}
