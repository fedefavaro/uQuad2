#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <path_planning.h>

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

    // Creo lista de trayectorias
    Lista_path *lista_path = (Lista_path *)malloc(sizeof(struct ListaIdentificar_path));
    inicializacion_path(lista_path);

    // Lista de way points para salida (trayectoria discreta)
    Lista_wp *output_wp = (Lista_wp *)malloc(sizeof(struct ListaIdentificar_wp));
    inicializacion_wp(output_wp);

    // Iteracion para hallar trayectoria por cada par de way points
    Elemento_wp *aux = lista_way_point->inicio;
    way_point_t *p_inicial = aux->dato;
    way_point_t *p_final = aux->siguiente->dato;
    for (i=0; i<cantidad_wp-1; i++)
    {
        // Guardo espacio para el par de way point a unir con una trayectoria
        way_point_t *p_inicial_conv = (way_point_t *)malloc(sizeof(struct way_point));
        way_point_t *p_final_conv = (way_point_t *)malloc(sizeof(struct way_point));

        // Convierto sistema de coordenadas
        conversion_eje_coordenadas(p_inicial, p_final, p_inicial_conv, p_final_conv);

        // Determino en que cuadrantes se encuentran los angulos de partida y llegada
        cuadrantes_t cuad = determinacion_cuadrantes(p_inicial_conv, p_final_conv);

        // Defino el tipo de trayectoria
        tipo_trayectoria_t *path_type = (tipo_trayectoria_t *)malloc(sizeof(enum tipo_trayectoria));
        eleccion_curva_dubins(cuad, p_inicial_conv, p_final_conv, path_type);

        // Hallo la trayectoria
        trayectoria_t *path = (trayectoria_t *)malloc(sizeof(struct trayectoria));
        find_path(*path_type, p_inicial, p_final, p_inicial_conv, p_final_conv, path);

        // Discretizo la trayectoria generada
        path_discreto(*path, output_wp);

        // Guardo la trayectoria en la lista
        InsercionEnLista_path(lista_path, *path);

        // Me muevo al proximo way point para el proximo ciclo del loop
        if (i<cantidad_wp-2)
        {
            aux = aux->siguiente;
            p_inicial = aux->dato;
            p_final = aux->siguiente->dato;
        }
    }

    // Genero log de trayectorias
    retval = log_resultado_trayectoria(lista_path);

    log_trayectoria_discreta(output_wp);

    return 0;
}

