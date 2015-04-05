#!/bin/sh -e

if [ ! -d "build" ]
then
  mkdir build
else
  rm -rf build/*
fi
cd build
cmake ..
cd ..
echo Done!

