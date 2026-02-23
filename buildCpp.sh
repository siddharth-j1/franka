#!/bin/bash

echo "Compiling C-GMS executable..."

g++ -O3 -Wall deploy_cgms.cpp -o run_cgms \
-I/home/siddharth/miniconda3/envs/franka/include \
-I/home/siddharth/miniconda3/envs/franka/include/eigen3 \
-L/home/siddharth/miniconda3/envs/franka/lib \
-lfranka \
-Wl,-rpath,/home/siddharth/miniconda3/envs/franka/lib \
-Wl,--disable-new-dtags

echo "Build complete! You can now execute ./run_cgms"
