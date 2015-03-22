#!/bin/sh -e

if [ ! -d "build" ]
then
  mkdir build
fi
cd build
cmake ..
cd ..
echo Done!

