#ifndef UQUAD_CONFIG_h
#define UQUAD_CONFIG_h

#define PC_TEST			0		

#if PC_TEST
   #define SBUS_LOG_TO_FILE	0  //sbus logea en un archivo en lugar de imprimir en stdout
#endif //PC_TEST

#define DEBUG                   1

#if DEBUG
   #define DEBUG_TIMING_MAIN	1 //imprime en stdout la duracion del loop
   #define DEBUG_TIMING_SBUSD	1
#endif //DEBUG






#endif //UQUAD_CONFIG_h
