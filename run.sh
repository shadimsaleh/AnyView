#!/bin/bash

clear
#cd ./results
# rm ./results/*.ppm
rm ./results/*.txt
cd ./build
make clean
make
cd ..
./build/ImageD ./data/pc_data/ $1 $2
