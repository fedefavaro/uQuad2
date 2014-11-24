#ifndef __ARDUINO_PRE_DECLARE_H__
#define __ARDUINO_PRE_DECLARE_H__


struct uquad_mat{
    double ** m;     // elements as [er,ec]: m[row][col]
    double * m_full; // elements as [er,ec]: m[m->c*er + ec]
    int r;           // rows
    int c;           // columns
};
typedef struct uquad_mat uquad_mat_t;



#endif // __ARDUINO_PRE_DECLARE_H___

