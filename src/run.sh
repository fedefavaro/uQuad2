#!/bin/bash -e

# Make sure only root can run our script
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

#trap ctrl_c INT
#function ctrl_c() {
#    echo "** Trapped CTRL-C"
#    killall sbusd
#    exit
#}

cd build/main
./main
cd ../..


# kill everything. Muaha, ha.
#ctrl_c
