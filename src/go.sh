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

if [ $1 ];
then
    log_name=$1
else
    echo No se especifico archivo de logeo, cerrando...
    exit
fi

if [ $2 ];
then
    throttle_inicial=$2
else
    echo No se especifico throttle inicial, cerrando...
    exit
fi

cd build/main
./auto_pilot ${log_name} ${throttle_inicial}
cd ../..


# kill everything
#ctrl_c
