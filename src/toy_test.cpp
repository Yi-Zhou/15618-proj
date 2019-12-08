#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include "mpi.h"

float max_diff(float diff) {
    int rank, n_procs;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &n_procs);
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
        //printf("rank %d diff %f i %d\n", rank, diff, i);
        MPI_Wait(&req, MPI_STATUS_IGNORE);
        diff = std::max(other_max_diff, diff);
    }

    return diff;
}


int main(int argc, char *argv[]) {
    MPI_Init(&argc, &argv);
    std::vector<float> diff {0.1, 0.3, 0.5, 0.4, 0.2, 0.8, 0.9, 0.6};
    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    // printf("max difference %f\n", max_diff(diff[rank]));
    MPI_Finalize();
    // Image& img = ReadImage(image);
    // std::shared_ptr<FactorGraph> fg = std::make_shared<FactorGraph>(img);
    // SynchronousBeliefPropagator bp(fg);

    //while (!converged) {
    // while (bp.beliefPropagate());
    // bp.merge(); TODO: calculate beliefs and merge the beliefs
        // check if converged
}