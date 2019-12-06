#include "common.h"
#include "fg.h"
#include "mpi.h"
#include <cassert>


#define SIZE_TAG 1
#define DATA_TAG 2

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
        //TODO: adjust parameters. I don't think 0.9 can work here
        edge_potential.x = Vec2<float>(1.0, 0.3);
        edge_potential.y = Vec2<float>(0.3, 1.0);
        // sub_graph = partition(fg)
        this->fg = fg;
        //partition(fg);
    }

    std::vector<Message> merge() {
        std::vector<Message> beliefs;

        // collect beliefs of variables in current partition
        for (auto vector : fg->variables) {
            for (std::shared_ptr<Variable> v : vector) {
                if (v->partition != rank) {
                    continue;
                }
                Vec2<float> belief = v->calulateBelief();
                Message msg;
                msg.position = v->position;
                msg.message = belief;
                msg.direction = 0; // can be ignored
                Vec2<float> norm_b = v->belief;
                //printf("sending normalized belief for v(%d, %d) is (%f, %f)\n", v->position.x, v->position.y,
                //norm_b.x, norm_b.y);

                beliefs.push_back(msg);
            }
        }

        //start to merge with other processors;
        int tag1 = SIZE_TAG;
        int tag2 = DATA_TAG;
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
            MPI_Irecv(&size, 1, MPI_INT, pair, tag1, MPI_COMM_WORLD, &req);
            MPI_Send(&curr_size, 1, MPI_INT, pair, tag1, MPI_COMM_WORLD);
            MPI_Wait(&req, MPI_STATUS_IGNORE);

            beliefs.resize(curr_size + size);

            //printf("beliefs size %d after resize %d\n", size, beliefs.size());
            MPI_Irecv(&beliefs[curr_size], sizeof(Message) * size, MPI_BYTE, pair, tag2, MPI_COMM_WORLD, &req);
            MPI_Send(&beliefs[0], sizeof(Message) * curr_size, MPI_BYTE, pair, tag2, MPI_COMM_WORLD);
            MPI_Wait(&req, MPI_STATUS_IGNORE);
            //printf("receiving normalized belief for v(%d, %d) is (%f, %f)\n", msg.position.x, msg.position.y,
            // msg.message.x, msg.message.y);
        }

        assert(beliefs.size() == fg->width * fg->height);
        return beliefs;
        // first calculate beliefs
    }

    float updateInMessages(std::vector<Message>& in_messages) {
        float max_diff = 0;
        for (Message msg : in_messages) {
            Vec2<int> p = msg.position;
            // printf("receive msg to (%d, %d) from direction %d\n", p.x, p.y, msg.direction);
            std::shared_ptr<Variable> v = fg->variables[p.x][p.y];

            Vec2<float> old_message = v->in_msgs[msg.direction];
            Vec2<float> diff = old_message - msg.message;
            //TODO: check whether normalizing in this way is correct or not
            max_diff = std::max(old_message.l1_norm(msg.message), max_diff);

            v->in_msgs[msg.direction] = msg.message;
        }
        return max_diff;
    }
    
    void getOutMessages(std::vector<std::vector<Message>>& out_messages) {
        // TODO: better way? to decrease time complexity
        for (auto vector : fg->variables) {
            for (std::shared_ptr<Variable> v : vector) {

                if (v->partition != rank) {
                    continue;
                }
                //printf("v(%d, %d) in partition %d\n", v->position.x, v->position.y, rank);

                Vec2<float> out_msg(1.0, 1.0);
                for (int i = 0; i < 5; i++) {
                    out_msg *= v->in_msgs[i];
                }

                for (int i = 0; i < 4; i++) {
                    std::shared_ptr<Variable> neighbor = v->neighbors[i];

                    if (neighbor != nullptr) {
                        Vec2<float> prod = out_msg / v->in_msgs[i];
                        int direction = (i + 2) % 4;
                        //printf("neighbor from %d\n", neighbor->partition);
                        Message msg;
                        msg.direction = direction;
                        msg.message = Vec2<float>(prod.x * edge_potential.x.x + prod.y * edge_potential.y.x,
                        prod.x * edge_potential.x.y + prod.y * edge_potential.y.y).normalize();

                        msg.position = Vec2<int>(neighbor->position.x, neighbor->position.y);
                        out_messages[neighbor->partition].push_back(msg);
                    }
                }

            }
        }
    }

    bool beliefPropagate() {
        std::vector<bool> neighbor_procs_set(n_procs, false);

        int tag1 = 1; // transmit a size
        int tag2 = 2; // transmit messages
        std::vector<MPI_Request> size_reqs;
        std::vector<MPI_Request> msg_reqs;

        std::vector<std::vector<Message>> out_messages(n_procs, std::vector<Message>());
        std::vector<Message> in_messages;

        getOutMessages(out_messages);
        for (int i = 0; i < n_procs; i++) {
            if (!out_messages[i].empty()) {
                //printf("out_messages to %d size %d for rank %d\n", i, out_messages[i].size(), rank);
                if (i == rank) {
                    in_messages.insert(in_messages.end(), 
                        out_messages[i].begin(),
                        out_messages[i].end());
                } else {
                    //MPI_Irec send
                    int size = out_messages[i].size();
                    // to decide tag
                    MPI_Request size_req, msg_req;
                    Message msg = out_messages[i][0];
                    //printf("sending msg (%f, %f) to (%d, %d) from direction %d from rank %d to i %d\n", 
                   // msg.message.x, msg.message.y, msg.position.x, msg.position.y, msg.direction, rank, i);
                    MPI_Isend(&size, 1, MPI_INT, i, tag1, MPI_COMM_WORLD, &size_req);
                   // MPI_Isend(&out_messages[i], size, message_dt, i, tag2, MPI_COMM_WORLD, &msg_req);
                    MPI_Isend(&out_messages[i][0], sizeof(Message) * size, MPI_BYTE, i, tag2, MPI_COMM_WORLD, &msg_req);
                   // MPI_Isend(&out_messages[i], size, message_dt, i, tag2, MPI_COMM_WORLD, &msg_req);
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
                //printf("%d ", i);
            }
        }

        int sizes[neighbor_procs.size()];
        for (int i = 0; i < neighbor_procs.size(); i++) {
            MPI_Request size_req;
            MPI_Irecv(&sizes[i], 1, MPI_INT, neighbor_procs[i], 
                        tag1, MPI_COMM_WORLD, &size_req);
            size_reqs.push_back(size_req);
        }

        MPI_Status stats[size_reqs.size()];
        //printf("size_reqs size %d for rank %d\n", size_reqs.size(), rank);
        assert(size_reqs.size() > 0);
        MPI_Waitall(size_reqs.size(), &size_reqs[0], stats);

        int total_size = 0;
        for (int i = 0; i < neighbor_procs.size(); i++) {
            total_size += sizes[i];
        }
        //printf("size_reqs.size %d total_size %d\n", size_reqs.size(), total_size);

        //Message inbox[total_size];
        int curr_size = in_messages.size();
        in_messages.resize(total_size + curr_size);
        for (int i = 0; i < neighbor_procs.size(); i++) {
            MPI_Request msg_req;
            MPI_Irecv(&in_messages[curr_size], sizeof(Message) * sizes[i], MPI_BYTE,
                    neighbor_procs[i], tag2, MPI_COMM_WORLD, &msg_req);
            // MPI_Recv(&in_messages[curr_size], sizes[i], message_dt,
            //          neighbor_procs[i], tag2, MPI_COMM_WORLD, &msg_req);
            msg_reqs.push_back(msg_req);
            curr_size += sizes[i];
        }

        assert(msg_reqs.size() == size_reqs.size());
        MPI_Waitall(msg_reqs.size(), &msg_reqs[0], stats);
        //printf("msg_reqs.size %d\n", msg_reqs.size());

        // TODO: Only For Debugging 
        for (Message msg : in_messages) {
            Vec2<int> p = msg.position;
            //printf("receive msg(%f, %f) to (%d, %d) from direction %d to rank %d\n", 
            //msg.message.x, msg.message.y,
            //p.x, p.y, msg.direction, rank);
            //printf("receive msg to %d from proc %d to rank %d\n", msg.direction, neighbor_procs[i], rank);
        }

        float diff = updateInMessages(in_messages);
        //printf("belief propgate diff %f rank %d\n", diff, rank);
        //return true;
        return is_converged(diff);
    }

private:
    bool is_converged(float diff) {
        // done test in toy_test
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
            MPI_Wait(&req, MPI_STATUS_IGNORE);
            diff = std::max(other_max_diff, diff);
        }

        printf("max diff %f rank %d\n", diff, rank);
        return diff < beta;
    }
};

int main(int argc, char *argv[]) {
    MPI_Init(&argc, &argv);
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

    Image img = Image::ReadImage("data/rice.txt");
    printf("img %d\n", img.pixels.size(), img.pixels[0].size());
    std::shared_ptr<FactorGraph> fg = std::make_shared<FactorGraph>(img.pixels, "data/256_256_8.txt");
    SynchronousBeliefPropagator bp(fg);

    int i = 0;
    while (!bp.beliefPropagate()) {
        if (i == 10) {
            break;
        }
        i++;
    }
    std::vector<Message> beliefs = bp.merge();
    int rank; 
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    if (rank == 0) {
        assert(beliefs.size() == img.w * img.h);
        FactorGraph::writeDenoisedImage(beliefs, "output/denoised_rice.txt");
        for (Message m : beliefs) {
            Vec2<float> norm_b = m.message.normalize();
            printf("normalized belief for v(%d, %d) is (%f, %f)\n", m.position.x, m.position.y,
            norm_b.x, norm_b.y);
        }
    }
    MPI_Finalize();
    // bp.merge(); TODO: calculate beliefs and merge the beliefs
        // check if converged
}



