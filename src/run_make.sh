#!/bin/sh -e

cd build/main
make
cd ../sbus_daemon
make
cd ../..
echo Done!

cp build/sbus_daemon/sbusd build/main/

