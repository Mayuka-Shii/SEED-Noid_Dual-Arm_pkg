#!/bin/sh

cd ../RTC/DualArmController/
mkdir build
cd build
cmake ..
make

cd ../../DualArmSimulation2.0/
mkdir build
cd build
cmake ..
make

