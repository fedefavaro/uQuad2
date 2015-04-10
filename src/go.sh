#!/bin/sh -e

# Make sure only root can run our script
#if [[ $EUID -ne 0 ]]; then
#   echo "This script must be run as root" 1>&2
#   exit 1
#fi

cd build/main
make
cd ../sbus_daemon
make
cd ../..
echo Done!

cp build/sbus_daemon/sbusd build/main/

#trap ctrl_c INT
#function ctrl_c() {
#    echo "** Trapped CTRL-C"
#    killall sbusd
#    exit
#}

cd build/main
./path_follower
cd ../..


# kill everything
#ctrl_c
