#ifndef PATH_PlANNING_H
#define PATH_PLANNING_H

#define pii 3.141592653589793238462643
#define RADIO 5
#define DISTANCIA_WP 0.5      // distancia entre dos way points de las trayectorias

typedef struct way_point {
    double x;
    double y;
    double z;
    double angulo;
} way_point_t;

typedef enum tipo_trayectoria {
    LSL = 0,
    LSR,    //1
    RSR,    //2
    RSL,    //3
    RLR,    //4
    LRL     //5
} tipo_trayectoria_t;

typedef enum cuadrantes {
    uno_uno = 0,
    uno_dos,        //1
    uno_tres,       //2
    uno_cuatro,     //3
    dos_uno,        //4
    dos_dos,        //5
    dos_tres,       //6
    dos_cuatro,     //7
    tres_uno,       //8
    tres_dos,       //9
    tres_tres,      //10
    tres_cuatro,    //11
    cuatro_uno,     //12
    cuatro_dos,     //13
    cuatro_tres,    //14
    cuatro_cuatro,  //15
} cuadrantes_t;     //16

typedef struct trayectoria {
    double xi;                  // x inicial
    double yi;                  // y inicial
    double zi;
    double anguloi;
    double xf;                  // x final
    double yf;                  // y final
    double zf;
    double angulof;
    tipo_trayectoria_t tipo;    // tipo de trayectoria
    double xci;                 // x centro cfa inicial
    double yci;                 // y centro cfa inicial
    double xcf;                 // x centro cfa final
    double ycf;                 // y centro cfa final
    double Ci;                  // angulo trayectoria primer circulo
    double S;                   // largo trayectoria recta
    double Cf;                  // angulo trayectoria segundo circulo
} trayectoria_t;


/** --------------------- */
/** LISTA PARA WAY POINTS */
/** --------------------- */

typedef struct ElementoLista_wp{
    way_point_t *dato;
    struct ElementoLista_wp *siguiente;
} Elemento_wp;

typedef struct ListaIdentificar_wp{
    Elemento_wp *inicio;
    Elemento_wp *fin;
    int tamano;
} Lista_wp;


/** ----------------------- */
/** LISTA PARA TRAYECTORIAS */
/** ----------------------- */

typedef struct ElementoLista_path{
    trayectoria_t *dato;
    struct ElementoLista_path *siguiente;
} Elemento_path;

typedef struct ListaIdentificar_path{
    Elemento_path *inicio;
    Elemento_path *fin;
    int tamano;
} Lista_path;






/**
 * Convierte un valor de angulo en grados
 * a radianes
 *
 * @param grados
 *
 * @return radianes
 */
double conversion_grados2rad(double grados);

/**
 * Devuelve el mismo angulo pero dentro
 * del rango 0 - 2pi
 *
 * @param angulo [rad]
 *
 * @return angulo [rad]
 */
double mod2pi(double angulo);

/**
 * Pide al usuario ingresar los way points
 * iniciales y finales para generar la
 * trayectoria
 *
 * @param p_inicial
 * @param p_final
 */
//void way_points_input(way_point_t *p_inicial, way_point_t *p_final);

void way_points_input(way_point_t *wp);

/**
 * Convierte los way point al sistema de
 * coordenadas a utilizar para el planeamiento
 * de trayectorias, donde el eje x coincide
 * con la recta que une los puntos de origen
 * y destino
 *
 * @param p_inicial
 * @param p_final
 */
 void conversion_eje_coordenadas(way_point_t *p_inicial_src, way_point_t *p_final_src, way_point_t *p_inicial_dest, way_point_t *p_final_dest);

/**
 * Determina en que cuadrantes se encuentran
 * los angulo de los way points iniciales y
 * finales
 *
 * @param p_inical
 * @param p_final
 *
 * @return cuadrantes
 */
cuadrantes_t determinacion_cuadrantes(way_point_t *p_inicial, way_point_t *p_final);

/**
 * Varias funciones que calculan las distancias
 * de los tramos de distintas trayectorias posibles
 *
 * Nomenclatura:    t - primer tramo
 *                  p - segundo tramo
 *                  q - tercer tramo
 *
 * @param a angulo alpha
 * @param b angulo beta
 * @param d distancia entre origen y destino
 *
 * @return largo del tramo de la trayectoria en cuestion
 */
double t_lsl(double a, double b, double d);
double p_lsl(double a, double b, double d);
double q_lsl(double a, double b, double d);
double t_rsr(double a, double b, double d);
double p_rsr(double a, double b, double d);
double q_rsr(double a, double b, double d);
double t_lsr(double a, double b, double d);
double p_lsr(double a, double b, double d);
double q_lsr(double a, double b, double d);
double t_rsl(double a, double b, double d);
double p_rsl(double a, double b, double d);
double q_rsl(double a, double b, double d);
double t_rlr(double a, double b, double d);
double p_rlr(double a, double b, double d);
double q_rlr(double a, double b, double d);
double t_lrl(double a, double b, double d);
double p_lrl(double a, double b, double d);
double q_lrl(double a, double b, double d);

/**
 * Define que tipo de curva de Dubins es
 * la que minimiza la trayectoria
 *
 * Posibles tipos de curvas:
 * LSL, LSR, RSR, RSL, RLR, LRL
 *
 * @param p_inicial
 * @param p_final
 * @param path_type
 */
void eleccion_curva_dubins(cuadrantes_t cuad, way_point_t *p_inicial, way_point_t *p_final, tipo_trayectoria_t *path_type);

/**
 */
void find_path(tipo_trayectoria_t tipo, way_point_t *p_inicial, way_point_t *p_final, way_point_t *p_inicial_conv, way_point_t *p_final_conv, trayectoria_t *path);

/**
 */
int log_resultado_trayectoria(Lista_path *lista);

/**
 */
int path_discreto(trayectoria_t trayectoria, Lista_wp *lista);

/**
 */
int log_trayectoria_discreta(Lista_wp *lista);

/** --------------------- */
/** LISTA PARA WAY POINTS */
/** --------------------- */

/*Inicializar una lista*/
void inicializacion_wp(Lista_wp *lista);

/*Inserción en una lista */
int InsercionEnLista_wp(Lista_wp *lista, way_point_t dato);

/*visualizar lista entera*/
void visualizacion_wp(Lista_wp *lista);



/** ----------------------- */
/** LISTA PARA TRAYECTORIAS */
/** ----------------------- */

/*Inicializar una lista*/
void inicializacion_path(Lista_path *lista);

/*Inserción en una lista */
int InsercionEnLista_path(Lista_path *lista, trayectoria_t dato);

/*visualizar lista entera*/
void visualizacion_path(Lista_path *lista);

#endif
