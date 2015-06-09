#!/bin/sh -e

if [ $1 ];
then
    log_name=$1
else
    echo No se especifico archivo de logeo, cerrando...
    exit
fi

cp build/main/${log_name} ../Matlab/Muestra_defensa/plot_datos_log/

chown labcontrol2 ../Matlab/Muestra_defensa/plot_datos_log//${log_name}

sudo chmod 664 ../Matlab/Muestra_defensa/plot_datos_log/${log_name}


