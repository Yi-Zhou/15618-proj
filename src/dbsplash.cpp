#include <math>
#include <vector>
#include "fg.h"
#include "mpi.h"
#include "common.h"
#include "mpi.h"

inline int updiv(int x, int y) {
  return (x + y - 1) / y;
}

struct deref_cmp {
  bool operator() (const std::shared_ptr<Variable> v1, 
                   const std::shared_ptr<Variable> v2) {
    return *v1 < *v2;
  }
}

struct Message {
  int direction;
  Vec2<float> message;
  std::shared_ptr<Variable> variable;
}

class DistributedBeliefPropagator {
public:
  std::priority_queue<std::shared_ptr<Variable>, 
                      std::vector<std::shared_ptr<Variable>,
                      deref_cmp> pq;
  int rank, n_procs;

  BeliefPropagator(std::shared_ptr<FactorGraph> fg, int bfs_depth) : fg(fg) {
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &n_procs);
    partition(fg);
  }

  void SendMessages(std::shared_ptr<Variable> v, 
                    std::vector<Message>& boundary_msgs) {
    // Update belief.
    Vec2 new_belief = Vec2<float>(1.0, 1.0)
    for (int i = 0; i < 4; i++) {
      int direction = (i + 2) % 4;
      std::shared_ptr<Variable> neighbor = neighbors[i];
      if (neighbor->partition == rank) {
        neighbor->in_
      } else {
        Message msg;
        msg.direction = (i + 2) % 4;
        msg.message = Vec2();
        msg.variable = neighbor;
        boundary_msgs.emplace_back(msg);
      }

    }
  }

  Splash(std::shared_ptr<Variable> v) {
    // Grow a spanning tree.
    std::vector<std::shared_ptr<Variable>>& ordered_variables = 
      ConstructBFSOrdering(v);
      std::vector<Message> boundary_msgs;
    // From the leaves to the root.
    for (auto it = ordered_variables.rbegin();
         it != ordered_variables.rend(); ++it) {
      SendMessages(v, boundary_msgs);
    }
    // From the root to the leaves.
    for (auto& v: ordered_variables) {
      SendMessages(v, boundary_msgs);
    }
    
    var = queue.pop_left();
    Vec2<float> out_msg(1.0, 1.0);
    for (int i = 0; i < 5; i++) {
      out_msg *= var->in_msgs[i];
    }
    for (int i = 0; i < 4; i++) {
      Vec2<float> msg = out_msg / var->in_msg[i];
      std::shared_ptr<Variable> neighbor = var->neighbors[i];
      if (neighbor.process == -1) {
        continue;
      }
      neighbor.in_msgs[(i+2)%4] = Vec2<float>(msg.x * 1.0 + msg.y * 0.9,
                                              msg.x * 0.9 + msg.y * 1.0);
      queue.pusb_back(neighbor);
    }
    // Calculate out messages for each variable.
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
          pq.push_back(var);
        }
      }
    }
  }
};

int main() {
  Image& img = ReadImage(image);
  std::shared_ptr<FactorGraph> fg = std::make_shared<FactorGraph>(img);
  DistributedBeliefPropagator dbp(fg);
}
