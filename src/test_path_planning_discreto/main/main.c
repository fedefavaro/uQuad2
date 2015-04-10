#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "path_planning.h"

int retval;
int i;

int main()
{
    // Ingreso de cantidad de way points por parte del usuario
    int cantidad_wp;
    printf("\nGENERACION DE TRAYECTORIAS\n");
    printf("------------------------------\n");
    inicio:
    printf("\nIngrese la cantidad de way points\t=\t");
    scanf("%i", &cantidad_wp);
    if (cantidad_wp < 2)
    {
        printf("\nAl menos debe ingresarse dos way points\n");
        goto inicio;
    }

    // Ingreso de los way points por parte del usuario
    Lista_wp *lista_way_point = (Lista_wp *)malloc(sizeof(struct ListaIdentificar_wp));
    inicializacion_wp(lista_way_point);
    way_point_t wp;
    for (i=0; i<cantidad_wp; i++)
    {
        way_points_input(&wp);
        retval = InsercionEnLista_wp(lista_way_point, wp);
        if (retval == -1)
            printf("\nERROR!\n");
    }

    path_planning(lista_way_point);

    return 0;
}

