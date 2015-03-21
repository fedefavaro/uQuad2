#!/bin/sh -e

echo Cleaning server build...
rm -rf test_msgq_server/build/
echo Done!
echo Cleaning client build...
rm test_msgq_client/test_msgq_client \
   test_msgq_client/main.o
echo Done!
