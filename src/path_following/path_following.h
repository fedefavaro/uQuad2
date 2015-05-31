#ifndef PATH_FOLLOWING_H
#define PATH_FOLLOWING_H

#include <path_planning.h>

#define DELTA               5        // Parametro seguimiento rectas [m]
#define LAMBDA              0.9      // Parametro seguimiento circunferencias [rad]
#define ERROR_ACEPTABLE     5        // Distancia minima para determinar si se logro el objetivo [m]

typedef enum estado {
    CFA_i,
    RECTA,
    CFA_f
} estado_t;

/**
 * Ejecuta el seguimiento de trayectorias
 * rectilineas, devolviendo el angulo yaw
 * deseado
 *
 * @param wp_i extremo inicial de la recta
 * @param wp_f extremo final de la recta
 * @param p way point de partida
 *
 * @return yaw deseado a seguir
 */
double carrotChase_Line(way_point_t wp_i, way_point_t wp_f, way_point_t p);

/**
 * Ejecuta el seguimiento de trayectorias
 * circulares, devolviendo el angulo yaw
 * deseado
 *
 * @param wp_c centro de la cfa
 * @param p way point de partida
 * @param sentido de giro en que recorre la cfa (izq o der)
 *
 * @return yaw deseado a seguir
 */
double carrotChase_Circle(way_point_t wp_c, int r, way_point_t p, char sentido);

int path_following(way_point_t p, Lista_path *lista_path, double *yaw_d);

#endif
