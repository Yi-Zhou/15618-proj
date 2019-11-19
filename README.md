# Distributed Belief Propagation using OpenMPI - Checkpoint Report

## Summary
So far, we have implemented data structures for representing a factor graph including the nodes and edges and a message which is passed within nodes. Also, we have implemented the algorithm to partition the factor graph into subgraphs. Each subgraph is assigned to a processor so that each processor is only responsible for belief propagation of its assigned subgraph and communicating the messages of the nodes on the boundary with other processors. Also, the implementations of synchronous belief propagation and the distributed residual Splash algorithm are already in progress.

## Goals
We swapped the order of implementing synchronous belief propagation and algorithms to partition the factor graph because we decided to implement our synchronous belief propagation algorithm in a distributed manner which also required partitioning a factor graph into subgraphs. Our implementation of synchronous belief propagation is a little bit behind our tentative schedule but our implementation of distributed splash algorithm is in fact ahead of our schedule. Therefore, we believe we can achieve our "plan to achieve" goal and hopefully, we can achieve our "hope to achieve" goal. 

## Poster Session Plan
At the poster session, we are going to show the denoising effects on different images using the implementation of both synchronous belief propagation and distributed residual Splash algorithms. We also plan to show the performance analysis of these two algorithms, like how much speedup that distributed residual Splash algorithm is over synchronous belief propagation. Also, if we successfully implement over-partitioning, we will show the experiment results of the distributed residual Splash algorithm with different hyperparameters.

## Preliminary Results
Since we haven't finished the implementation of our parallel belief propagation algorithms, we don't preniminary results up to now.

## Issues
The major issue we are concerned about is whether our implemented residual splash algorithm could have a good speedup over the synchronous belief propagation. Also, debugging a large-scale MPI program can be tricky and time-consuming. 

## Updated Tentative Schedule
11/23: Finish implementation of synchronous belief propagation - Shuaijun Ge
11/23: Finish implementation of distributed Residual Splash algorithm - Yi Zhou
11/27: Test implemented synchronous belief propagation on datasets - Shuaijun Ge
11/27: Test implemented distributed Residual Splash algorithm on datasets - Yi Zhou
12/1: Finish implementation of over-partitioning - Shuaijun Ge, Yi Zhou
12/6: Test over-partitioning implementation and tune hyperparameters to do experiments - Shuaijun Ge, Yi Zhou
12/9: Finish final project report - Shuaijun Ge, Yi Zhou


