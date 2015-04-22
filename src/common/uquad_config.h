#ifndef UQUAD_CONFIG_h
#define UQUAD_CONFIG_h

#define PC_TEST			1	

#if PC_TEST
   #define SBUS_LOG_TO_FILE	1  //sbus logea en un archivo en lugar de imprimir en stdout
#endif //PC_TEST

#define DEBUG                   0

#if DEBUG
   #define DEBUG_TIMING_MAIN	1 //imprime en stdout la duracion del loop
   #define DEBUG_TIMING_SBUSD	0
#endif //DEBUG






#endif //UQUAD_CONFIG_h
