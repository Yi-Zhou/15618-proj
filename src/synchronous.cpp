
#include "fg.h"
#include "mpi.h"
#include <cassert>

class SynchronousBeliefPropagator {
public:
    double beta;
    int rank, n_procs;
    Vec2<Vec2<float>> edge_potential; 
    std::shared_ptr<FactorGraph> fg;
    std::vector<int> neighbor_procs;

    SynchronousBeliefPropagator(std::shared_ptr<FactorGraph> fg) {
        beta = 1e-5; // TODO: change a value
        MPI_Comm_rank(MPI_COMM_WORLD, &rank);
        MPI_Comm_size(MPI_COMM_WORLD, &n_procs);
        edge_potential.x = Vec2<float>(1.0, 0.9);
        edge_potential.y = Vec2<float>(0.9, 1.0);
        // sub_graph = partition(fg)
        this->fg = fg;
        //partition(fg);
    }

    float updateInMessages(std::vector<Message> in_messages) {
        float max_diff = 0;
        for (Message msg : in_messages) {
            Vec2<int> p = msg.position;
            std::shared_ptr<Variable> v = fg->variables[p.x][p.y];

            Vec2<float> old_message = v->in_msgs[msg.direction];
            Vec2<float> diff_x = old_message - msg.message;
            max_diff = std::max(std::abs(diff_x.x) + std::abs(diff_x.y), max_diff);

            v->in_msgs[msg.direction] = msg.message;
        }
        return max_diff;
    }
    
    void getOutMessages(std::vector<std::vector<Message>> out_messages) {
        for (auto vector : fg->variables) {
            for (std::shared_ptr<Variable> v : vector) {

                if (v->rank != rank) {
                    continue;
                }

                Vec2<float> out_msg(1.0, 1.0);
                for (int i = 0; i < 5; i++) {
                    out_msg *= v->in_msgs[i];
                }

                for (int i = 0; i < 4; i++) {
                    Vec2<float> prod = out_msg / v->in_msgs[i];
                    int direction = (i + 2) % 4;
                    std::shared_ptr<Variable> neighbor = v->neighbors[i];

                    if (neighbor != nullptr) {
                        int direction = (i + 2) % 4;
                        
                        Message msg;
                        msg.direction = direction;
                        msg.message = Vec2<float>(prod.x * edge_potential.x.x + prod.y * edge_potential.x.y,
                        prod.x * edge_potential.y.x + prod.y * edge_potential.y.y);

                        msg.position = Vec2<int>(neighbor->position.x, neighbor->position.y);
                        out_messages[neighbor->rank].push_back(msg);
                    }
                }

            }
        }
    }

    bool beliefPropagate() {
        MPI_Aint displacements[3] = {offsetof(Message, direction), 
                                    offsetof(Message, message), 
                                    offsetof(Message, position)};
                                    
        
        int block_lengths[3] = {1, 2, 2};
        MPI_Datatype types[3] = {MPI_INT, MPI_FLOAT, MPI_INT};
        MPI_Datatype message_dt;

        MPI_Type_create_struct(3, block_lengths, displacements, types, 
                                &message_dt);
        MPI_Type_commit(&message_dt);
        std::vector<bool> neighbor_procs_set(n_procs, false);

        int tag1 = 1; // transmit a size
        int tag2 = 2; // transmit messages
        std::vector<MPI_Request> size_reqs;
        std::vector<MPI_Request> msg_reqs;

        std::vector<std::vector<Message>> out_messages(n_procs, std::vector<Message>());
        getOutMessages(out_messages);
        std::vector<Message> in_messages;
        for (int i = 0; i < n_procs; i++) {
            if (!out_messages[i].empty()) {
                if (i == rank) {
                    in_messages.insert(in_messages.end(), 
                        out_messages[i].begin(),
                        out_messages[i].end());
                } else {
                    //MPI_Irec send
                    int size = out_messages[i].size();
                    // to decide tag
                    MPI_Request size_req, msg_req;
                    MPI_Isend(&size, 1, MPI_INT, i, tag1, MPI_COMM_WORLD, &size_req);
                    MPI_Isend(&out_messages[i], size, message_dt, i, tag2, MPI_COMM_WORLD, &msg_req);
                    size_reqs.push_back(size_req);
                    msg_reqs.push_back(msg_req);
                    neighbor_procs_set[i] = true;
                }
            }
        }

        std::vector<int> neighbor_procs;
        for (int i = 0; i < n_procs; i++) {
            if (neighbor_procs_set[i]) {
                neighbor_procs.push_back(i);
            }
        }

        int sizes[neighbor_procs.size()];
        for (int i = 0; i < neighbor_procs.size(); i++) {
            MPI_Request size_req;
            MPI_Irecv(&sizes[i], sizeof(int), MPI_INT, neighbor_procs[i], 
                        tag1, MPI_COMM_WORLD, &size_req);
            size_reqs.push_back(size_req);
        }

        //TODO: may not compile
        MPI_Status stats[size_reqs.size()];
        MPI_Waitall(size_reqs.size(), &size_reqs[0], stats);

        for (int i = 0; i < neighbor_procs.size(); i++) {
            MPI_Request msg_req;
            MPI_Irecv(&in_messages[in_messages.size() - 1], sizes[i], message_dt,
                    neighbor_procs[i], tag2, MPI_COMM_WORLD, &msg_req);
        }

        assert(msg_reqs.size() == size_reqs.size());
        MPI_Waitall(msg_reqs.size(), &msg_reqs[0], stats);

        float diff = updateInMessages(in_messages);
        return is_converged(diff);
    }

private:
    bool is_converged(float diff) {
        MPI_Request req;
        int tag1 = 1;

        for (int i = 1; i < n_procs; i <<= 1) {
            int pair;
            if (rank % (i << 1) < i) {
                pair = rank + i;
            } else {
                pair = rank - i;
            }
            float other_max_diff;
            MPI_Irecv(&other_max_diff, 1, MPI_FLOAT, pair, tag1, MPI_COMM_WORLD, &req);
            MPI_Send(&diff, 1, MPI_FLOAT, pair, tag1, MPI_COMM_WORLD);
            diff = std::max(other_max_diff, diff);
            MPI_Wait(&req, MPI_STATUS_IGNORE);
        }

        return diff < beta;
    }
};

int main(int argc, char *argv[]) {
    MPI_Init(&argc, &argv);
    // Image& img = ReadImage(image);
    // std::shared_ptr<FactorGraph> fg = std::make_shared<FactorGraph>(img);
    // SynchronousBeliefPropagator bp(fg);

    //while (!converged) {
    // while (bp.beliefPropagate());
    // bp.merge(); TODO: calculate beliefs and merge the beliefs
        // check if converged
}



