#include <cstddef> /* NULL */
#include <metis.h>
#include <iostream> 
#include <fstream> 
#include "common.h"


// Install metis from:
// http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/metis-5.1.0.tar.gz

// Build with
// g++ metis.cc -lmetis

int example(){

    idx_t nVertices = 6;
    idx_t nEdges    = 7;
    idx_t nWeights  = 1;
    idx_t nParts    = 2;

    idx_t objval;
    idx_t part[nVertices];


    // Indexes of starting points in adjacent array
    idx_t xadj[6+1] = {0,2,5,7,9,12,14};

    // Adjacent vertices in consecutive index order
    idx_t adjncy[2 * 7] = {1,3,0,4,2,1,5,0,4,3,1,5,4,2};

    // Weights of vertices
    // if all weights are equal then can be set to NULL
    idx_t vwgt[nVertices * nWeights];
    

    // int ret = METIS_PartGraphRecursive(&nVertices,& nWeights, xadj, adjncy,
    // 				       NULL, NULL, NULL, &nParts, NULL,
    // 				       NULL, NULL, &objval, part);

    int ret = METIS_PartGraphKway(&nVertices,& nWeights, xadj, adjncy,
				       NULL, NULL, NULL, &nParts, NULL,
				       NULL, NULL, &objval, part);

    std::cout << ret << std::endl;
    
    for(unsigned part_i = 0; part_i < nVertices; part_i++){
	std::cout << part_i << " " << part[part_i] << std::endl;
    }

    
    return 0;
}


class MetisImage {
public:
    int width, height;

    MetisImage(int w, int h) {
        width = w;
        height = h;
    }
   
    void partition(int n_parts, const char* result) {
        // |---w----
        // h
        // |
        idx_t total_nodes = width * height;
        idx_t edges = (width - 1) * (height - 1) * 2 + width - 1 + height - 1;
        idx_t n_weights  = 1;

        idx_t objval;
        idx_t xadj[total_nodes + 1];
        idx_t adjncy[2 * edges];
        idx_t part[total_nodes];

        int ai = 0;
        xadj[0] = 0;
        int xi = 1;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int idx = positionToIdx(i, j);
                printf("idx %d\n", idx);
                //UP
                if (i > 0) {
                    adjncy[ai++] = positionToIdx(i-1, j);
                }
                //LEFT
                if (j > 0) {
                    adjncy[ai++] = positionToIdx(i, j-1);
                }
                //RIGHT
                if (j < width - 1) {
                    adjncy[ai++] = positionToIdx(i, j+1);
                }
                //DOWN
                if (i < height - 1) {
                    adjncy[ai++] = positionToIdx(i+1, j);
                }
                xadj[xi++] = ai;
            }
        }
        printf("xi %d ai %d\n", xi, ai);
        printf("xadj: ");
        for (int i = 0; i < total_nodes + 1; i++) {
            printf("%d ", xadj[i]);
        }
        printf("\nadjncy: ");
        for (int i = 0; i < 2 * edges; i++) {
            printf("%d ", adjncy[i]);
        }
        printf("\n");

        int ret = METIS_PartGraphKway(&total_nodes, &n_weights, xadj, adjncy,
				       NULL, NULL, NULL, &n_parts, NULL,
				       NULL, NULL, &objval, part);

        std::cout << ret << std::endl;
    
        std::ofstream output;
        output.open(result);

        for(unsigned part_i = 0; part_i < total_nodes; part_i++){
            Vec2<int> p = idxToPosition(part_i);
	        std::cout << part_i << " " << part[part_i] << std::endl;
            output << part[part_i] << std::endl;
        }

        output.close();
    }
private:
    int positionToIdx(int x, int y) {
       return x * width + y; 
    }

    Vec2<int> idxToPosition(int idx) {
        return Vec2<int>(idx / width, idx % width);
    }

};

int main() {
    MetisImage img(256, 256);
    img.partition(8, "data/256_256_8.txt");
}

