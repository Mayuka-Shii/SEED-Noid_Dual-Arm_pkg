#!/bin/sh

#SeedUpperBody
cd ../RTC/SeedUpperBody/
mkdir build
cd build
cmake ..
make

#SeedDualArmController
cd ../../SeedDualArmController/
mkdir build
cd build
cmake ..
make

