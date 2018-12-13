#!/bin/sh

cd ../RTC/SeedUpperBody2.0/
mkdir build
cd build
cmake ..
make

cd ../../DualArmController/
mkdir build
cd build
cmake ..
make

