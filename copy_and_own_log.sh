#!/bin/sh -e

if [ $1 ];
then
    log_name=$1
else
    echo No se especifico archivo de logeo, cerrando...
    exit
fi

if [ $2 ];
then
    folder_dest=$2
else
    echo No se especifico carpeta destino, cerrando...
    exit
fi

cp build/main/${log_name} ~/uQuad2/logs/DEBUG/${folder_dest}

chown labcontrol2 ~/uQuad2/logs/DEBUG/${folder_dest}/${log_name}

sudo chmod 664 ~/uQuad2/logs/DEBUG/${folder_dest}/${log_name}
