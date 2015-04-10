#include <stdio.h>
#include <math.h>
#include "path_planning.h"
#include <stdlib.h>
#include <complex.h>

double conversion_grados2rad(double grados)
{
    return grados*pii/180;
}

double mod2pi(double angulo)
{
    double retval = angulo;
    while (retval >= 2*pii)
        retval = retval - 2*pii;
    while (retval < 0)
        retval = retval + 2*pii;

    return retval;
}

//void way_points_input(way_point_t *p_inicial, way_point_t *p_final)
//{
//    printf("Ingreso de datos: way points\n");
//    printf("----------------------------\n");
//    printf("\nWay point inicial:\nx\t=\t");
//    scanf("%lf", &p_inicial->x);
//    printf("y\t=\t");
//    scanf("%lf", &p_inicial->y);
//    printf("z\t=\t");
//    scanf("%lf", &p_inicial->z);
//    printf("angulo\t=\t");
//    scanf("%lf", &p_inicial->angulo);
//
//    printf("\nWay point final:\nx\t=\t");
//    scanf("%lf", &p_final->x);
//    printf("y\t=\t");
//    scanf("%lf", &p_final->y);
//    printf("z\t=\t");
//    scanf("%lf", &p_final->z);
//    printf("angulo\t=\t");
//    scanf("%lf", &p_final->angulo);
//
//    return;
//}

void way_points_input(way_point_t *wp)
{
    printf("\nWay point:\nx\t=\t");
    scanf("%lf", &wp->x);
    printf("y\t=\t");
    scanf("%lf", &wp->y);
    printf("z\t=\t");
    scanf("%lf", &wp->z);
    printf("angulo\t=\t");
    scanf("%lf", &wp->angulo);

    wp->angulo = mod2pi(conversion_grados2rad(wp->angulo));

    return;
}



void conversion_eje_coordenadas(way_point_t *p_inicial_src, way_point_t *p_final_src, way_point_t *p_inicial_dest, way_point_t *p_final_dest)
{
    double aux = atan2(p_final_src->y - p_inicial_src->y, p_final_src->x - p_inicial_src->x);

    p_inicial_dest->x = 0;
    p_inicial_dest->y = 0;
    p_inicial_dest->z = 0;
    p_inicial_dest->angulo = mod2pi(p_inicial_src->angulo - aux);

    p_final_dest->x = sqrt(pow(p_final_src->x - p_inicial_src->x, 2) + pow(p_final_src->y - p_inicial_src->y, 2));
    p_final_dest->y = 0;
    p_final_dest->z = 0;
    p_final_dest->angulo = mod2pi(p_final_src->angulo - aux);

    return;
}



cuadrantes_t determinacion_cuadrantes(way_point_t *p_inicial, way_point_t *p_final)
{
    cuadrantes_t cuad;

    if ((0 <= p_inicial->angulo) && (p_inicial->angulo < pii/2)) {
        if ((0 <= p_final->angulo) && (p_final->angulo < pii/2))
            cuad = uno_uno;
        else if ((pii/2 <= p_final->angulo) && (p_final->angulo < pii))
            cuad = uno_dos;
        else if ((pii <= p_final->angulo) && (p_final->angulo < 3*pii/2))
            cuad = uno_tres;
        else if ((3*pii/2 <= p_final->angulo) && (p_final->angulo < 2*pii))
            cuad = uno_cuatro;
    }

    if ((pii/2 <= p_inicial->angulo) && (p_inicial->angulo < pii)) {
        if ((0 <= p_final->angulo) && (p_final->angulo < pii/2))
            cuad = dos_uno;
        else if ((pii/2 <= p_final->angulo) && (p_final->angulo < pii))
            cuad = dos_dos;
        else if ((pii <= p_final->angulo) && (p_final->angulo < 3*pii/2))
            cuad = dos_tres;
        else if ((3*pii/2 <= p_final->angulo) && (p_final->angulo < 2*pii))
            cuad = dos_cuatro;
    }

    if ((pii <= p_inicial->angulo) && (p_inicial->angulo < 3*pii/2)) {
        if ((0 <= p_final->angulo) && (p_final->angulo < pii/2))
            cuad = tres_uno;
        else if ((pii/2 <= p_final->angulo) && (p_final->angulo < pii))
            cuad = tres_dos;
        else if ((pii <= p_final->angulo) && (p_final->angulo < 3*pii/2))
            cuad = tres_tres;
        else if ((3*pii/2 <= p_final->angulo) && (p_final->angulo < 2*pii))
            cuad = tres_cuatro;
    }

    if ((3*pii/2 <= p_inicial->angulo) && (p_inicial->angulo < 2*pii)) {
        if ((0 <= p_final->angulo) && (p_final->angulo < pii/2))
            cuad = cuatro_uno;
        else if ((pii/2 <= p_final->angulo) && (p_final->angulo < pii))
            cuad = cuatro_dos;
        else if ((pii <= p_final->angulo) && (p_final->angulo < 3*pii/2))
            cuad = cuatro_tres;
        else if ((3*pii/2 <= p_final->angulo) && (p_final->angulo < 2*pii))
            cuad = cuatro_cuatro;
    }

    return cuad;
}



/** LSL */
double t_lsl(double a, double b, double d)
{
    return -a + mod2pi(atan2(cos(b) - cos(a), d + sin(a) - sin(b)));
}
double p_lsl(double a, double b, double d)
{
    return sqrt(2 + pow(d,2) - (2*cos(a - b)) + (2*d*(sin(a) - sin(b))));
}
double q_lsl(double a, double b, double d)
{
    return b - mod2pi(atan2(cos(b) - cos(a), d + sin(a) - sin(b)));
}

/** RSR */
double t_rsr(double a, double b, double d)
{
    return a - mod2pi(atan2(cos(a) - cos(b), d - sin(a) + sin(b)));
}
double p_rsr(double a, double b, double d)
{
    return sqrt(2 + pow(d,2) - (2*cos(a - b)) + (2*d*(sin(b) - sin(a))));
}
double q_rsr(double a, double b, double d)
{
    return -mod2pi(b) + mod2pi(atan2(cos(a) - cos(b), d - sin(a) + sin(b)));
}

/** LSR */
double t_lsr(double a, double b, double d)
{
    return mod2pi(-a + atan2(-cos(a) - cos(b), d + sin(a) + sin(b)) - atan2(-2, p_lsr(a,b,d)));
}
double p_lsr(double a, double b, double d)
{
    return sqrt(-2 + pow(d,2) + (2*cos(a - b)) + (2*d*(sin(a) + sin(b))));
}
double q_lsr(double a, double b, double d)
{
    return -mod2pi(b) + atan2(-cos(a) - cos(b), d + sin(a) + sin(b)) - mod2pi(atan2(-2, p_lsr(a,b,d)));
}

/** RSL */
double t_rsl(double a, double b, double d)
{
    return a - atan2(cos(a) + cos(b), d - sin(a) - sin(b)) + mod2pi(atan2(2, p_rsl(a,b,d)));
}
double p_rsl(double a, double b, double d)
{
    return sqrt(pow(d,2) -2 + (2*cos(a - b)) - (2*d*(sin(a) + sin(b))));
}
double q_rsl(double a, double b, double d)
{
    return mod2pi(b) - atan2(cos(a) + cos(b), d - sin(a) - sin(b)) + mod2pi(atan2(2, p_rsl(a,b,d)));
}

/** RLR */
double t_rlr(double a, double b, double d)
{
    return a - atan2(cos(a) - cos(b), d - sin(a) + sin(b)) + mod2pi(p_rlr(a,b,d));
}
double p_rlr(double a, double b, double d)
{
    return acos((6 - pow(d,2) + (2*cos(a - b)) + (2*d*(sin(a) - sin(b))))/8);
}
double q_rlr(double a, double b, double d)
{
    return a - b - t_rlr(a,b,d) + mod2pi(p_rlr(a,b,d));
}

/** LRL */
double t_lrl(double a, double b, double d)
{
    return mod2pi(-a + atan2(-cos(a) + cos(b), d + sin(a) - sin(b)) + (p_lrl(a,b,d)/2));
}
double p_lrl(double a, double b, double d)
{
    return mod2pi(acos((6 - pow(d,2) + (2*cos(a - b)) + (2*d*(sin(a) - sin(b))))/8));
}
double q_lrl(double a, double b, double d)
{
    return mod2pi(b) - a + (2*mod2pi(p_lrl(a,b,d)));
}

double fun_f(double i, double j, double k)
{
    return i - j - (2*(k - pii));
}

double fun_g(double i)
{
    return i- pii;
}



void eleccion_curva_dubins(cuadrantes_t cuad, way_point_t *p_inicial, way_point_t *p_final, tipo_trayectoria_t *path_type)
{
    double a = p_inicial->angulo;
    double b = p_final->angulo;
    double d = p_final->x/RADIO;

    switch (cuad) {

        case uno_uno:
            *path_type = RSL;
            break;

        case uno_dos:
            if (fun_f(p_rsr(a,b,d), p_rsl(a,b,d), q_rsl(a,b,d)) < 0)
                *path_type = RSR;
            else
                *path_type = RSL;
            break;

        case uno_tres:
            if (fun_g(t_rsr(a,b,d)) < 0)
                *path_type = RSR;
            else
                *path_type = LSR;
            break;

        case uno_cuatro:
            if (fun_g(t_rsr(a,b,d)) > 0)
                *path_type = LSR;
            else if (fun_g(q_rsr(a,b,d)) > 0)
                *path_type = RSL;
            else
                *path_type = RSR;
            break;

        case dos_uno:
            if (fun_f(p_lsl(a,b,d), p_rsl(a,b,d), t_rsl(a,b,d)) < 0)
                *path_type = LSL;
            else
                *path_type = RSL;
            break;

        case dos_dos:
            if (a > b) {
                if (fun_f(p_lsl(a,b,d), p_rsl(a,b,d), t_rsl(a,b,d)) < 0)
                    *path_type = LSL;
                else
                    *path_type = RSL;
            } else {
                if (fun_f(p_rsr(a,b,d), p_rsl(a,b,d), q_rsl(a,b,d)) < 0)
                    *path_type = RSR;
                else
                    *path_type = RSL;
            }
            break;

        case dos_tres:
            *path_type = RSR;
            break;

        case dos_cuatro:
            if (fun_g(q_rsr(a,b,d)) < 0)
                *path_type = RSR;
            else
                *path_type = RSL;
            break;

        case tres_uno:
            if (fun_g(q_lsl(a,b,d)) < 0)
                *path_type = LSL;
            else
                *path_type = LSR;
            break;

        case tres_dos:
            *path_type = LSL;
            break;

        case tres_tres:
            if (a < b) {
                if (fun_f(p_rsr(a,b,d), p_lsr(a,b,d), t_lsr(a,b,d)) < 0)
                    *path_type = RSR;
                else
                    *path_type = LSR;
            } else {
                if (fun_f(p_lsl(a,b,d), p_lsr(a,b,d), q_lsr(a,b,d)) < 0)
                    *path_type = LSL;
                else
                    *path_type = LSR;
            }
            break;

        case tres_cuatro:
            if (fun_f(p_rsr(a,b,d), p_lsr(a,b,d), t_lsr(a,b,d)) < 0)
                *path_type = RSR;
            else
                *path_type = LSR;
            break;

        case cuatro_uno:
            if (fun_g(t_lsl(a,b,d)) > 0)
                *path_type = RSL;
            else if (fun_g(q_lsl(a,b,d)) > 0)
                *path_type = LSR;
            else
                *path_type = LSL;
            break;

        case cuatro_dos:
            if (fun_g(t_lsl(a,b,d)) < 0)
                *path_type = LSL;
            else
                *path_type = RSL;
            break;

        case cuatro_tres:
            if (fun_f(p_lsl(a,b,d), p_lsr(a,b,d), q_lsr(a,b,d)) < 0)
                *path_type = LSL;
            else
                *path_type = LSR;
            break;

        case cuatro_cuatro:
            *path_type = LSR;
            break;

        default:
            printf("\nERROR: Error al buscar el tipo de trayectoria mas corta\n");
            break;
    }

    return;
}



void find_path(tipo_trayectoria_t tipo, way_point_t *p_inicial, way_point_t *p_final, way_point_t *p_inicial_conv, way_point_t *p_final_conv, trayectoria_t *path)
{
    path->xi = p_inicial->x;
    path->yi = p_inicial->y;
    path->zi = p_inicial->z;
    path->anguloi = p_inicial->angulo;
    path->xf = p_final->x;
    path->yf = p_final->y;
    path->zf = p_final->z;
    path->angulof = p_final->angulo;
    path->tipo = tipo;

    double a = p_inicial_conv->angulo;
    double b = p_final_conv->angulo;
    double d = p_final_conv->x/RADIO;

    // Cfa inicial
    double theta_inicial;
    if ((tipo == RSR) || (tipo == RSL) || (tipo == RLR))  // Si empieza hacia la derecha
        theta_inicial = mod2pi(p_inicial->angulo - (pii/2));
    else  // Si empieza hacia la izquierda
        theta_inicial = mod2pi(p_inicial->angulo + (pii/2));
    path->xci = p_inicial->x + (RADIO*cos(theta_inicial));
    path->yci = p_inicial->y + (RADIO*sin(theta_inicial));

    // Cfa final
    double theta_final;
    if ((tipo == RSR) || (tipo == RLR) || (tipo == LSR))  // Si empieza hacia la derecha
        theta_final = mod2pi(p_final->angulo - (pii/2));
    else  // Si empieza hacia la izquierda
        theta_final = mod2pi(p_final->angulo + (pii/2));
    path->xcf = p_final->x + (RADIO*cos(theta_final));
    path->ycf = p_final->y + (RADIO*sin(theta_final));

    switch (tipo) {
        case LSL:
            path->Ci = t_lsl(a,b,d);
            path->S = p_lsl(a,b,d);
            path->Cf = q_lsl(a,b,d);
            break;
        case LSR:
            path->Ci = t_lsr(a,b,d);
            path->S = p_lsr(a,b,d);
            path->Cf = q_lsr(a,b,d);
            break;
        case RSR:
            path->Ci = t_rsr(a,b,d);
            path->S = p_rsr(a,b,d);
            path->Cf = q_rsr(a,b,d);
            break;
        case RSL:
            path->Ci = t_rsl(a,b,d);
            path->S = p_rsl(a,b,d);
            path->Cf = q_rsl(a,b,d);
            break;
    }

    return;
}



int log_resultado_trayectoria(Lista_path *lista)
{
    Elemento_path *elemento = lista->inicio;

    // Creo archivo log file
    FILE *file_log;
    file_log = fopen("path_log.txt", "w");
    if (file_log == NULL)
    {
        return -1;
    }

    fprintf(file_log, "%i \n\n", lista->tamano);

    fprintf(file_log, "%i \n", elemento->dato->tipo);
    fprintf(file_log, "%lf \n", elemento->dato->xi);
    fprintf(file_log, "%lf \n", elemento->dato->yi);
    fprintf(file_log, "%lf \n", elemento->dato->zi);
    fprintf(file_log, "%lf \n", elemento->dato->anguloi);
    fprintf(file_log, "%lf \n", elemento->dato->xf);
    fprintf(file_log, "%lf \n", elemento->dato->yf);
    fprintf(file_log, "%lf \n", elemento->dato->zf);
    fprintf(file_log, "%lf \n", elemento->dato->angulof);
    fprintf(file_log, "%lf \n", elemento->dato->xci);
    fprintf(file_log, "%lf \n", elemento->dato->yci);
    fprintf(file_log, "%lf \n", elemento->dato->xcf);
    fprintf(file_log, "%lf \n", elemento->dato->ycf);
    fprintf(file_log, "%lf \n", elemento->dato->Ci);
    fprintf(file_log, "%lf \n", elemento->dato->S);
    fprintf(file_log, "%lf \n", elemento->dato->Cf);
    fprintf(file_log, "%i \n\n", RADIO);

     // Guardo resultados en log file
     while (elemento->siguiente != NULL) {

        elemento = elemento->siguiente;

        fprintf(file_log, "%i \n", elemento->dato->tipo);
        fprintf(file_log, "%lf \n", elemento->dato->xi);
        fprintf(file_log, "%lf \n", elemento->dato->yi);
        fprintf(file_log, "%lf \n", elemento->dato->zi);
        fprintf(file_log, "%lf \n", elemento->dato->anguloi);
        fprintf(file_log, "%lf \n", elemento->dato->xf);
        fprintf(file_log, "%lf \n", elemento->dato->yf);
        fprintf(file_log, "%lf \n", elemento->dato->zf);
        fprintf(file_log, "%lf \n", elemento->dato->angulof);
        fprintf(file_log, "%lf \n", elemento->dato->xci);
        fprintf(file_log, "%lf \n", elemento->dato->yci);
        fprintf(file_log, "%lf \n", elemento->dato->xcf);
        fprintf(file_log, "%lf \n", elemento->dato->ycf);
        fprintf(file_log, "%lf \n", elemento->dato->Ci);
        fprintf(file_log, "%lf \n", elemento->dato->S);
        fprintf(file_log, "%lf \n", elemento->dato->Cf);
        fprintf(file_log, "%i \n\n", RADIO);

    }

    // Cierro archivo de log file
    fclose(file_log);

    return 0;
}




/* discretiza la trayectoria */
int path_discreto(trayectoria_t trayectoria, Lista_wp *lista)
{
    double delta_ang = DISTANCIA_WP/RADIO;
    int i;

    double ang_inicial = mod2pi(atan2(trayectoria.yi - trayectoria.yci, trayectoria.xi - trayectoria.xci));
    double ang_final = mod2pi(atan2(trayectoria.yf - trayectoria.ycf, trayectoria.xf - trayectoria.xcf));

    way_point_t aux;
    double ang_aux;

    /* PRIMERA CFA */
    // si primera cfa es en sentido antihorario...
    if ((trayectoria.tipo == LSL) || (trayectoria.tipo == LSR) || (trayectoria.tipo == LRL)) {
        ang_aux = ang_inicial;
        for (i=0; i<cabs(mod2pi(trayectoria.Ci)/delta_ang); i++) {
            aux.x = trayectoria.xci + RADIO*cos(ang_aux + delta_ang);
            aux.y = trayectoria.yci + RADIO*sin(ang_aux + delta_ang);
            InsercionEnLista_wp(lista, aux);
            ang_aux = ang_aux + delta_ang;
        }
        aux.x = trayectoria.xci + RADIO*cos(ang_inicial + cabs(mod2pi(trayectoria.Ci)));
        aux.y = trayectoria.yci + RADIO*sin(ang_inicial + cabs(mod2pi(trayectoria.Ci)));
        InsercionEnLista_wp(lista, aux);
    } else { // si primera cfa es en sentido horario...
        ang_aux = ang_inicial;
        for (i=0; i<cabs(mod2pi(trayectoria.Ci)/delta_ang); i++) {
            aux.x = trayectoria.xci + RADIO*cos(ang_aux - delta_ang);
            aux.y = trayectoria.yci + RADIO*sin(ang_aux - delta_ang);
            InsercionEnLista_wp(lista, aux);
            ang_aux -= delta_ang;
        }
        aux.x = trayectoria.xci + RADIO*cos(ang_inicial - cabs(mod2pi(trayectoria.Ci)));
        aux.y = trayectoria.yci + RADIO*sin(ang_inicial - cabs(mod2pi(trayectoria.Ci)));
        InsercionEnLista_wp(lista, aux);
    }

    /* RECTA */
    double x_1, y_1, x_2, y_2;  // punto inicial y final de la recta
    if ((trayectoria.tipo == LSL) || (trayectoria.tipo == LSR) || (trayectoria.tipo == LRL)) {
        x_1 = trayectoria.xci + RADIO*cos(mod2pi(ang_inicial + cabs(mod2pi(trayectoria.Ci))));
        y_1 = trayectoria.yci + RADIO*sin(mod2pi(ang_inicial + cabs(mod2pi(trayectoria.Ci))));
    } else {
        x_1 = trayectoria.xci + RADIO*cos(mod2pi(ang_inicial - cabs(mod2pi(trayectoria.Ci))));
        y_1 = trayectoria.yci + RADIO*sin(mod2pi(ang_inicial - cabs(mod2pi(trayectoria.Ci))));
    }
    if ((trayectoria.tipo == LSL) || (trayectoria.tipo == RSL) || (trayectoria.tipo == LRL)) {
        x_2 = trayectoria.xcf + RADIO*cos(mod2pi(ang_final - cabs(mod2pi(trayectoria.Cf))));
        y_2 = trayectoria.ycf + RADIO*sin(mod2pi(ang_final - cabs(mod2pi(trayectoria.Cf))));
    } else {
        x_2 = trayectoria.xcf + RADIO*cos(mod2pi(ang_final + cabs(mod2pi(trayectoria.Cf))));
        y_2 = trayectoria.ycf + RADIO*sin(mod2pi(ang_final + cabs(mod2pi(trayectoria.Cf))));
    }
    double delta_x, delta_y;
    if (x_2-x_1==0)
        delta_x = 0;
    else
        delta_x = (DISTANCIA_WP/(x_2-x_1))*cos(atan2((y_2-y_1),(x_2-x_1)));
    if (y_2-y_1==0)
        delta_y = 0;
    else
        delta_y = (DISTANCIA_WP/(y_2-y_1))*sin(atan2((y_2-y_1),(x_2-x_1)));
    for (i=0; (i*delta_x<1) && (i*delta_y<1); i++) {
        aux.x = (x_2 - x_1)*i*delta_x + x_1;
        aux.y = (y_2 - y_1)*i*delta_y + y_1;
        InsercionEnLista_wp(lista, aux);
    }

    /* SEGUNDA CFA */
    // si ultima cfa es en sentido antihorario...
    if ((trayectoria.tipo == LSL) || (trayectoria.tipo == RSL) || (trayectoria.tipo == LRL)) {
        ang_aux = mod2pi(ang_final - cabs(mod2pi(trayectoria.Cf)));
        for (i=0; i<cabs(mod2pi(trayectoria.Cf)/delta_ang); i++) {
            aux.x = trayectoria.xcf + RADIO*cos(ang_aux + delta_ang);
            aux.y = trayectoria.ycf + RADIO*sin(ang_aux + delta_ang);
            InsercionEnLista_wp(lista, aux);
            ang_aux += delta_ang;
        }
        aux.x = trayectoria.xf;
        aux.y = trayectoria.yf;
        InsercionEnLista_wp(lista, aux);
    } else { // si ultima cfa es en sentido horario...
        ang_aux = mod2pi(ang_final + cabs(mod2pi(trayectoria.Cf)));
        for (i=0; i<cabs(mod2pi(trayectoria.Cf)/delta_ang); i++) {
            aux.x = trayectoria.xcf + RADIO*cos(ang_aux - delta_ang);
            aux.y = trayectoria.ycf + RADIO*sin(ang_aux - delta_ang);
            InsercionEnLista_wp(lista, aux);
            ang_aux -= delta_ang;
        }
        aux.x = trayectoria.xf;
        aux.y = trayectoria.yf;
        InsercionEnLista_wp(lista, aux);
    }

    return 0;
}



int log_trayectoria_discreta(Lista_wp *lista)
{
    Elemento_wp *elemento = lista->inicio;

    // Creo archivo log file
    FILE *output_file;
    output_file = fopen("trayectoria_discreta.txt", "w");
    if (output_file == NULL)
    {
        return -1;
    }

    fprintf(output_file, "%lf \t", elemento->dato->x);
    fprintf(output_file, "%lf \n", elemento->dato->y);

    // Guardo resultados en log file
     while (elemento->siguiente != NULL) {
        elemento = elemento->siguiente;
        fprintf(output_file, "%lf \t", elemento->dato->x);
        fprintf(output_file, "%lf \n", elemento->dato->y);
    }

    // Cierro archivo de log file
    fclose(output_file);

    return 0;
}



/** --------------------- */
/** LISTA PARA WAY POINTS */
/** --------------------- */

/*Inicializar una lista*/
void inicializacion_wp(Lista_wp *lista)
{
    lista->inicio = NULL;
    lista->fin= NULL;
    lista->tamano = 0;
}

/*Insercion en una lista */
int InsercionEnLista_wp(Lista_wp *lista, way_point_t dato)
{
    Elemento_wp *nuevo_elemento;
    if((nuevo_elemento = (Elemento_wp *)malloc(sizeof(Elemento_wp)))==NULL)
        return -1;
    if((nuevo_elemento->dato = (way_point_t *)malloc(sizeof(struct way_point)))==NULL)
        return -1;

    nuevo_elemento->dato->x = dato.x;
    nuevo_elemento->dato->y = dato.y;
    nuevo_elemento->dato->z = dato.z;
    nuevo_elemento->dato->angulo = dato.angulo;
    nuevo_elemento->siguiente = NULL;

    if (lista->tamano == 0)
        lista->inicio = nuevo_elemento;
    else
        lista->fin->siguiente = nuevo_elemento;
    lista->fin = nuevo_elemento;
    lista->tamano++;

    return 0;
}

/*visualizar lista entera*/
void visualizacion_wp(Lista_wp *lista)
{
    Elemento_wp *actual;
    actual = lista->inicio;
    while(actual != NULL) {
        printf("\n x = %lf",actual->dato->x);
        printf("\n y = %lf",actual->dato->y);
        printf("\n z = %lf",actual->dato->z);
        printf("\n angulo = %lf\n",actual->dato->angulo);
        actual = actual->siguiente;
    }

    return;
}



/** ----------------------- */
/** LISTA PARA TRAYECTORIAS */
/** ----------------------- */

/*Inicializar una lista*/
void inicializacion_path(Lista_path *lista)
{
    lista->inicio = NULL;
    lista->fin= NULL;
    lista->tamano = 0;
}

/*Insercion en una lista */
int InsercionEnLista_path(Lista_path *lista, trayectoria_t dato)
{
    Elemento_path *nuevo_elemento;
    if((nuevo_elemento = (Elemento_path *)malloc(sizeof(Elemento_path)))==NULL)
        return -1;
    if((nuevo_elemento->dato = (trayectoria_t *)malloc(sizeof(struct trayectoria)))==NULL)
        return -1;

    nuevo_elemento->dato->xi = dato.xi;
    nuevo_elemento->dato->yi = dato.yi;
    nuevo_elemento->dato->zi = dato.zi;
    nuevo_elemento->dato->anguloi = dato.anguloi;
    nuevo_elemento->dato->xf = dato.xf;
    nuevo_elemento->dato->yf = dato.yf;
    nuevo_elemento->dato->zf = dato.zf;
    nuevo_elemento->dato->angulof = dato.angulof;
    nuevo_elemento->dato->xci = dato.xci;
    nuevo_elemento->dato->yci = dato.yci;
    nuevo_elemento->dato->xcf = dato.xcf;
    nuevo_elemento->dato->ycf = dato.ycf;
    nuevo_elemento->dato->Ci = dato.Ci;
    nuevo_elemento->dato->Cf = dato.Cf;
    nuevo_elemento->dato->S = dato.S;
    nuevo_elemento->dato->tipo = dato.tipo;
    nuevo_elemento->siguiente = NULL;

    if (lista->tamano == 0)
        lista->inicio = nuevo_elemento;
    else
        lista->fin->siguiente = nuevo_elemento;
    lista->fin = nuevo_elemento;
    lista->tamano++;

    return 0;
}

/*visualizar lista entera*/
void visualizacion_path(Lista_path *lista)
{
    Elemento_path *actual;
    actual = lista->inicio;
    while(actual != NULL) {
        printf("\n x = %lf",actual->dato->xi);
        printf("\n y = %lf",actual->dato->yi);
        printf("\n z = %lf",actual->dato->zi);
        printf("\n x = %lf",actual->dato->xf);
        printf("\n y = %lf",actual->dato->yf);
        printf("\n z = %lf",actual->dato->zf);
        printf("\n x = %lf",actual->dato->xci);
        printf("\n y = %lf",actual->dato->yci);
        printf("\n x = %lf",actual->dato->xcf);
        printf("\n y = %lf",actual->dato->ycf);
        printf("\n x = %lf",actual->dato->Ci);
        printf("\n y = %lf",actual->dato->Cf);
        printf("\n x = %lf",actual->dato->S);
        printf("\n y = %d",actual->dato->tipo);
        actual = actual->siguiente;
    }

    return;
}
