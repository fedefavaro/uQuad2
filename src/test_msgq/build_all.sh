#!/bin/sh -e

echo Building server...
cd test_msgq_server
if [ ! -d "build" ]
then
  mkdir build
fi
cd build
cmake ..
cd main
make
echo Done building server!
cd ../../..

echo Now building client...
cd test_msgq_client
make
cp test_msgq_client ../test_msgq_server/build/main
cd ..
echo Done building client!


