#include "path_following.h"

#include <stdlib.h>
#include <math.h>

#define fix(a)                    ((a>0)?floor(a):ceil(a))
#define sign(a)                   ((a < 0.0)?-1.0:1.0)

static estado_t estado = CFA_i;
static double last_yaw_d = 0; //TODO primer caso me preocupa, como inicializo esta variable

double carrotChase_Line(way_point_t wp_i, way_point_t wp_f, way_point_t p)
{
    // Paso 2
    double Ru = sqrt(pow(wp_i.x - p.x, 2) + pow(wp_i.y - p.y, 2));
    double theta = atan2(wp_f.y - wp_i.y, wp_f.x - wp_i.x);

    // Paso 3
    double theta_u = atan2(p.y - wp_i.y, p.x - wp_i.x);
    double beta = theta - theta_u;

    // Paso 4
    double R = sqrt(pow(Ru, 2) - pow(Ru*sin(beta), 2));

    // Paso 5
    double x = wp_i.x + (R+DELTA)*cos(theta);
    double y = wp_i.y + (R+DELTA)*sin(theta);

    // Paso 6
    //double yaw_d = mod2pi(atan2(y - p.y, x - p.x));
    double yaw_d = -atan2(y - p.y, x - p.x); //El signo negativo es para que sea coherente con el sentido de giro de la cc3d (angulo positivo = giro horario)
    
    //Correccion de discontinuidad de atan2
    double dyaw_d = yaw_d - last_yaw_d;
    if (abs(dyaw_d) >= M_PI)
	yaw_d -= 2.0*M_PI*fix((dyaw_d+M_PI*sign(dyaw_d))/(2.0*M_PI));
    last_yaw_d = yaw_d;

    return yaw_d;
}

double carrotChase_Circle(way_point_t wp_c, int r, way_point_t p, char sentido)
{
    // Paso 2
    double d = sqrt(pow(wp_c.x - p.x, 2) + pow(wp_c.y - p.y, 2)) - r;

    // Paso 3
    double theta = atan2(p.y - wp_c.y, p.x - wp_c.x);

    // Paso 4
    double x, y;
    // Depende el sentido de giro el signo que uso para el angulo lambda
    if (sentido == 'L') {
        x = wp_c.x + r*cos(theta + LAMBDA);
        y = wp_c.y + r*sin(theta + LAMBDA);
    } else {
        x = wp_c.x + r*cos(theta - LAMBDA);
        y = wp_c.y + r*sin(theta - LAMBDA);
    }

    // Paso 5
    //double yaw_d = mod2pi(atan2(y - p.y, x - p.x));
    double yaw_d = -atan2(y - p.y, x - p.x); //El signo negativo es para que sea coherente con el sentido de giro de la cc3d (angulo positivo = giro horario)
    
    //Correccion de discontinuidad de atan2
    double dyaw_d = yaw_d - last_yaw_d;
    if (abs(dyaw_d) >= M_PI)
	yaw_d -= 2.0*M_PI*fix((dyaw_d+M_PI*sign(dyaw_d))/(2.0*M_PI));
    last_yaw_d = yaw_d;

    return yaw_d;
}

int path_following(way_point_t p, Lista_path *lista_path, double *yaw_d)
{
    int a,b,c;

    way_point_t centro, pi, pf;
    trayectoria_t path;

    inicioPathFollowing:;// printf("   ");    // TODO: Hacer que esto ande sin el printf()

    if (lista_path->tamano == 0)
        return -1;
    else
        path = *lista_path->inicio->dato;

    // Verifico si llegue al final de alguna subtrayectoria
    switch(estado) {

    case CFA_i:
        if (sqrt(pow(p.x - path.xri, 2) + pow(p.y - path.yri, 2)) < ERROR_ACEPTABLE)
            estado = RECTA;
        a=1;
        break;

    case RECTA:
        if (sqrt(pow(p.x - path.xrf, 2) + pow(p.y - path.yrf, 2)) < ERROR_ACEPTABLE) {
            estado = CFA_f;
            goto inicioPathFollowing;
        }
        a=2;
        break;

    case CFA_f:
        if (sqrt(pow(p.x - path.xf, 2) + pow(p.y - path.yf, 2)) < ERROR_ACEPTABLE) {
            BorrarEnLista_path(lista_path);
            estado = CFA_i;
            goto inicioPathFollowing;
        }
        a=3;
        break;

    }

    // dependiendo del caso hago el procedimiento de carrot chase
    switch(estado) {

    case CFA_i:
        centro.x = path.xci;
        centro.y = path.yci;
        if ((path.tipo == RSR) || (path.tipo == RSL) || (path.tipo == RLR)) {  // si empieza hacia la derecha
            *yaw_d = carrotChase_Circle(centro, RADIO, p, 'R');
        } else {    // si empieza hacia la izquierda
            *yaw_d = carrotChase_Circle(centro, RADIO, p, 'L');
        }
        b=1;
        break;

    case RECTA:
        pi.x = path.xri;
        pi.y = path.yri;
        pf.x = path.xrf;
        pf.y = path.yrf;
        *yaw_d = carrotChase_Line(pi, pf, p);
        b=2;
        break;

    case CFA_f:
        centro.x = path.xcf;
        centro.y = path.ycf;
        if ((path.tipo == RSR) || (path.tipo == LSR) || (path.tipo == RLR)) {  // si termina hacia la derecha
            *yaw_d = carrotChase_Circle(centro, RADIO, p, 'R');
        } else {    // si termina hacia la izquierda
            *yaw_d = carrotChase_Circle(centro, RADIO, p, 'L');
        }
        b=3;
        break;

    }

//    printf("\n %i   %i\n",a,b);
    return 0;

}
