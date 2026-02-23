#!/bin/bash

# Check if a filename was provided
if [ -z "$1" ]; then
    echo "Usage: ./buildCpp.sh <filename.cpp>"
    exit 1
fi

# Extract the filename without extension for the output binary
SOURCE_FILE=$1
OUTPUT_BIN="${SOURCE_FILE%.*}"

echo "Compiling $SOURCE_FILE into $OUTPUT_BIN..."

g++ -O3 -Wall "$SOURCE_FILE" -o "$OUTPUT_BIN" \
-I/home/siddharth/miniconda3/envs/franka/include \
-I/home/siddharth/miniconda3/envs/franka/include/eigen3 \
-L/home/siddharth/miniconda3/envs/franka/lib \
-lfranka \
-Wl,-rpath,/home/siddharth/miniconda3/envs/franka/lib \
-Wl,--disable-new-dtags

if [ $? -eq 0 ]; then
    echo "Build complete! You can now run: ./$OUTPUT_BIN"
else
    echo "Build failed."
fi
