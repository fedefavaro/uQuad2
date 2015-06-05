#ifndef PATH_PlANNING_H
#define PATH_PLANNING_H

#define WAYPOINTS_FILE	"way_points_in.txt"

#define pii 3.141592653589793238462643
#define RADIO 10.0                // Radio minimo de curvatura
//#define DISTANCIA_WP 0.5        // Distancia entre dos way points de la trayectoria

/** --------------------- */
/**      ESTRUCTURAS      */
/** --------------------- */

typedef struct way_point {
    double x;
    double y;
    double z;
    double angulo;
} way_point_t;

typedef enum tipo_trayectoria {
    LSL = 0,    // Left - Straight - Left
    LSR,        // Left - Straight - Right
    RSR,        // Right - Straight - Right
    RSL,        // Right - Straight - Left
    RLR,        // Right - Left - Right
    LRL         // Left - Right - Left
} tipo_trayectoria_t;

typedef enum cuadrantes {
    uno_uno = 0,
    uno_dos,
    uno_tres,
    uno_cuatro,
    dos_uno,
    dos_dos,
    dos_tres,
    dos_cuatro,
    tres_uno,
    tres_dos,
    tres_tres,
    tres_cuatro,
    cuatro_uno,
    cuatro_dos,
    cuatro_tres,
    cuatro_cuatro,
} cuadrantes_t;

typedef struct trayectoria {
    double xi;                  // x inicial
    double yi;                  // y inicial
    double zi;                  // z inicial
    double anguloi;             // angulo YAW inicial
    double xf;                  // x final
    double yf;                  // y final
    double zf;                  // z final
    double angulof;             // angulo YAW final
    tipo_trayectoria_t tipo;    // tipo de trayectoria
    double xri;                 // x inicio recta
    double yri;                 // y inicio recta
    double xrf;                 // x final recta
    double yrf;                 // y final recta
    double xci;                 // x centro cfa inicial
    double yci;                 // y centro cfa inicial
    double xcf;                 // x centro cfa final
    double ycf;                 // y centro cfa final
    double Ci;                  // angulo trayectoria primer circulo
    double S;                   // largo trayectoria recta (dividio el RADIO)
    double Cf;                  // angulo trayectoria segundo circulo
} trayectoria_t;



/** --------------------------------------------------------- */
/**                   LISTA DE WAY POINTS                     */
/** --------------------------------------------------------- */

typedef struct ElementoLista_wp{
    way_point_t *dato;
    struct ElementoLista_wp *siguiente;
} Elemento_wp;

typedef struct ListaIdentificar_wp{
    Elemento_wp *inicio;
    Elemento_wp *fin;
    int tamano;
} Lista_wp;

/* Inicializar una lista */
void inicializacion_wp(Lista_wp *lista);

/* Agregar en una lista */
int InsercionEnLista_wp(Lista_wp *lista, way_point_t dato);

/* visualizar lista entera */
void visualizacion_wp(Lista_wp *lista);

/** --------------------------------------------------------- */
/**                  LISTA DE TRAYECTORIAS                    */
/** --------------------------------------------------------- */

typedef struct ElementoLista_path{
    trayectoria_t *dato;
    struct ElementoLista_path *siguiente;
} Elemento_path;

typedef struct ListaIdentificar_path{
    Elemento_path *inicio;
    Elemento_path *fin;
    int tamano;
} Lista_path;

/* Inicializar una lista */
void inicializacion_path(Lista_path *lista);

/* Agregar en una lista */
int InsercionEnLista_path(Lista_path *lista, trayectoria_t dato);

/*Elimina primer elemento en una lista */
int BorrarEnLista_path(Lista_path *lista);

/* visualizar lista entera */
void visualizacion_path(Lista_path *lista);

/** --------------------------------------------------------- */
/** --------------------------------------------------------- */


/** --------------------- */
/**       FUNCIONES       */
/** --------------------- */

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

//**
// * Pide al usuario ingresar los way points
// * iniciales y finales para generar la
// * trayectoria
// * @param p_inicial
// * @param p_final
// */
//void way_points_input(way_point_t *p_inicial, way_point_t *p_final);

/**
 * Carga lista de way points desde
 * archivo de texto dado por el usuario
 */
int way_points_input(Lista_wp *wp_lista);

/**
 * Convierte los way point al sistema de
 * coordenadas a utilizar para el planeamiento
 * de trayectorias, donde el way point de salida
 * es el origen y el eje x coincide con la recta
 * que une ambos way points
 *
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
 * A partir del tipo de curva de Dubins y de
 * los way points iniciales y finales en ambos
 * sistemas de coordenadas, definen los datos
 * de la trayactoria
 *
 * x inicial
 * y inicial
 * z inicial
 * angulo YAW inicial
 * x final
 * y final
 * z final
 * angulo YAW final
 * tipo de trayectoria
 * x centro cfa inicial
 * y centro cfa inicial
 * x centro cfa final
 * y centro cfa final
 * angulo trayectoria primer circulo
 * largo trayectoria recta (dividio el RADIO)
 * angulo trayectoria segundo circulo
 */
void find_path(tipo_trayectoria_t tipo, way_point_t *p_inicial, way_point_t *p_final, way_point_t *p_inicial_conv, way_point_t *p_final_conv, trayectoria_t *path);

/**
 * Dada una trayectoria ya definida genera
 * una lista con la trayectoria discretizada
 * solo en terminos de posicion (x,y,z)
 */
int path_discreto(trayectoria_t trayectoria, Lista_wp *lista);

/**
 * Loggea en un archivo de texto .txt una lista
 * de way points (x,y,z)
 */
int log_trayectoria_discreta(Lista_wp *lista);

/**
 * Funcion principal que haciendo uso de todo el resto
 * genera un archivo .txt con una trayectoria discreta
 * a seguir
 */
int path_planning(Lista_wp *wp, Lista_path *lista_path);

#endif
