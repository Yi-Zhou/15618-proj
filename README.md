run make all to compile synchronous and dbspash belief propagation implementation
run make sync to only compile synchronous
run make dbs to only compile dbsplash
run make partition to only compile partition which uses METIS to partition a graph and write into a file (METIS required. Need to first install METIS. You find the package and install instructions for METIS from http://glaros.dtc.umn.edu/gkhome/metis/metis/overview).

To run synchronous implementation for noisy_rice.png denoising task with 8 processors, you can do
mpirun -np 8 build/synchronous 1000 data/noisy_rice.txt data/256_256_8.txt output/denoised_rice_8.png
TO run synchronous implementation for a noisy_fractal_triangles.png denoising task with 8 processors, you can do
mpirun -np 8 build/synchronous 1000 data/noisy_fractal_triangles.txt data/2048_2048_8.txt output/denoised_fractal_triangles_8.png

You can find all noisy and original images under the data directory and all denoised images under output directories

The metis.ipynb contains some helper functions like adding noises to help up preprocess our images.

