/**
 * uquad_kalman: lib for EKF.
 * Copyright (C) 2012  Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file   uquad_kalman.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for EKF.
 *
 */

#include <math.h>
#include "uquad_kalman.h"
/*#include "uquad_types.h"*/
#include "uquad_config.h"

/*/// Vars for inertial kalman*/
struct uquad_mat* Fk_1_inertial = NULL;
struct uquad_mat* Fk_1_T_inertial = NULL;
/*struct uquad_mat* tmp_inertial = NULL;*/
/*struct uquad_mat* tmp2_inertial = NULL;*/
struct uquad_mat* mtmp_inertial = NULL;
struct uquad_mat* H_inertial = NULL;
struct uquad_mat* fx_inertial = NULL;
struct uquad_mat* hx_inertial = NULL;
/*// Auxiliares Update*/
struct uquad_mat* yk_inertial = NULL;
struct uquad_mat* HT_inertial = NULL;
struct uquad_mat* HP__inertial = NULL;
struct uquad_mat* HP_HT_inertial = NULL;
struct uquad_mat* Sk_inertial = NULL;
struct uquad_mat* Sk_aux1_inertial = NULL;
struct uquad_mat* Sk_aux2_inertial = NULL;
struct uquad_mat* P_HT_inertial = NULL;
struct uquad_mat* Kk_inertial = NULL;
struct uquad_mat* Kkyk_inertial = NULL;
struct uquad_mat* KkH_inertial = NULL;
struct uquad_mat* IKH_inertial = NULL;
struct uquad_mat* Sk_1_inertial = NULL;
struct uquad_mat* I_inertial = NULL;

/*/// Var for drag/drive*/
struct uquad_mat* TM       = NULL;
struct uquad_mat* D        = NULL;
struct uquad_mat* w_2      = NULL;
struct uquad_mat* w_2_tmp  = NULL;
struct uquad_mat* w_2_tmp2 = NULL;

/*// Auxiliaries for Update*/
struct uquad_mat* fx_gps = NULL;
struct uquad_mat* Fk_1_gps = NULL;
struct uquad_mat* Fk_1_T_gps = NULL;
struct uquad_mat* mtmp_gps = NULL;

/*// Auxiliaries for Update*/
struct uquad_mat* hx_gps = NULL;
struct uquad_mat* yk_gps = NULL;
struct uquad_mat* H_gps = NULL;
struct uquad_mat* HP__gps  = NULL;
struct uquad_mat* HT_gps   = NULL;
struct uquad_mat* HP_HT_gps = NULL;
struct uquad_mat* Sk_gps   = NULL;
struct uquad_mat* Sk_aux1_gps = NULL;
struct uquad_mat* Sk_aux2_gps = NULL;
struct uquad_mat* Sk_1_gps = NULL;
struct uquad_mat* P_HT_gps = NULL;
struct uquad_mat* Kk_gps   = NULL;
struct uquad_mat* Kkyk_gps = NULL;
/*struct uquad_mat* x_hat_gps = NULL;*/
struct uquad_mat* I_gps    = NULL;
struct uquad_mat* KkH_gps  = NULL;
struct uquad_mat* IKH_gps  = NULL;

int H_init(struct uquad_mat** H, uquad_bool_t is_gps)
{
    int retval;
    if(!is_gps)
	*H = uquad_mat_alloc(KALMAN_ROWS_H,STATE_COUNT+STATE_BIAS);
    else
	*H = uquad_mat_alloc(KALMAN_ROWS_H_GPS,STATE_COUNT+STATE_BIAS);
    if(*H == NULL)
    {
	err_check(ERROR_MALLOC, "Failed to allocate H()!");
    }
    retval = uquad_mat_zeros(*H);
    err_propagate(retval);
    (*H)->m[0][3]=1;
    (*H)->m[1][4]=1;
    (*H)->m[2][5]=1;
#if KALMAN_BIAS
    (*H)->m[3][12]=1;
    (*H)->m[4][13]=1;
    (*H)->m[5][14]=1;
#endif // KALMAN_BIAS
    (*H)->m[6][9]=1;
    (*H)->m[7][10]=1;
    (*H)->m[8][11]=1;
    (*H)->m[9][2]=1;
    if(is_gps)
    {
	/// Reorder last terms, use {x,y,z}
	(*H)->m[9][2]  = 0;
	(*H)->m[9][0]  = 1;
	(*H)->m[10][1] = 1;
	(*H)->m[11][2] = 1;
    }
    return ERROR_OK;
}

/*int store_data(kalman_io_t* kd, struct uquad_mat *w, imu_data_t* data, double T, double weight, gps_comm_data_t *gps_i_dat)*/
/*{*/
/*    struct uquad_mat *z = (gps_i_dat == NULL)?*/
/*	kd->z:*/
/*	kd->z_gps;*/
/*    int retval;*/
/*    retval = uquad_mat_copy(kd->u, w);*/
/*    err_propagate(retval);*/

/*    retval = uquad_mat_set_subm(z, 0, 0, data->magn);*/
/*    err_propagate(retval);*/

/*    retval = uquad_mat_set_subm(z, 3, 0, data->acc);*/
/*    err_propagate(retval);*/

/*    retval = uquad_mat_set_subm(z, 6, 0, data->gyro);*/
/*    err_propagate(retval);*/

/*    if(gps_i_dat != NULL)*/
/*    {*/
/*	retval = uquad_mat_set_subm(z, 9, 0, gps_i_dat->pos);*/
/*	err_propagate(retval);*/
/*	/* GPS ALTITUDE INFORMATION IS IGNORED */
/*	z->m_full[11] = data->alt;*/
/*    }*/
/*    else*/
/*    {*/
/*	z->m_full[9] = data->alt;*/
/*    }*/

/*    kd->T = T/1000000;*/
/*    kd->weight = weight;*/

/*    return ERROR_OK;*/
/*}*/

/*/***/
/* * Calculates drag() torque, using:*/
/* *    drag = w^2*DRAG_A2 + w*DRAG_A1*/
/* **/
/* * NOTE: Aux mem can be either all NULL, or all of size (LENGTH_INPUTx1)*/
/* **/
/* * @param drag answer (LENGTH_INPUTx1)*/
/* * @param w input (LENGTH_INPUTx1)*/
/* * @param w_2 aux mem or NULL*/
/* * @param w_2_tmp aux mem or NULL*/
/* * @param w_2_tmp2 aux mem or NULL*/
/* **/
/* * @return error code*/
/* */
int drag(struct uquad_mat *drag, struct uquad_mat *w,
	 struct uquad_mat *w_2,
	 struct uquad_mat *w_2_tmp,
	 struct uquad_mat *w_2_tmp2)
{  
    int retval;
    uquad_bool_t local_mem = false;
    if(w_2 == NULL || w_2_tmp == NULL || w_2_tmp2 == NULL)
    {
	/**
	 * Global vars used for tmp memory.
	 * If they are not defined, define them.
	 */
	if(w_2 != NULL || w_2_tmp != NULL || w_2_tmp2 != NULL)
	{
	    err_check(ERROR_INVALID_ARG,"Aux mem must be all or none!");
	}
	w_2      = uquad_mat_alloc(LENGTH_INPUT,1);
	w_2_tmp  = uquad_mat_alloc(LENGTH_INPUT,1);
	w_2_tmp2 = uquad_mat_alloc(LENGTH_INPUT,1);
	local_mem = true;
	if((w_2 == NULL) || (w_2_tmp == NULL) || (w_2_tmp2 == NULL))
	{
	    cleanup_log_if(ERROR_MALLOC, "Failed to allocate mem for drag()!");
	}
    }
    retval = uquad_mat_dot_product(w_2,w,w);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(w_2_tmp, w_2, DRAG_A1);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(w_2_tmp2, w, DRAG_A2);
    err_propagate(retval);
    retval = uquad_mat_add(drag,w_2_tmp,w_2_tmp2);
    err_propagate(retval);
    cleanup:
    if(local_mem)
    {
	uquad_mat_free(w_2);
	uquad_mat_free(w_2_tmp);
	uquad_mat_free(w_2_tmp2);
    }
    return retval;
}

/*/***/
/* * Calculates drive using:*/
/* *    drive = w^2*DRIVE_A2 + w*DRIVE_A1*/
/* **/
/* * NOTE: Aux mem can be either all NULL, or all of size (LENGTH_INPUTx1)*/
/* **/
/* * @param drive answer*/
/* * @param w input*/
/* * @param w_2 aux mem or NULL*/
/* * @param w_2_tmp aux mem or NULL*/
/* * @param w_2_tmp2 aux mem or NULL*/
/* **/
/* * @return error code*/
/* */
int drive(struct uquad_mat *drive, struct uquad_mat *w,
	 struct uquad_mat *w_2,
	 struct uquad_mat *w_2_tmp,
	 struct uquad_mat *w_2_tmp2)
{  
    int retval;
    uquad_bool_t local_mem = false;
    if(w_2 == NULL || w_2_tmp == NULL || w_2_tmp2 == NULL)
    {
	/**
	 * Global vars used for tmp memory.
	 * If they are not defined, define them.
	 */
	if(w_2 != NULL || w_2_tmp != NULL || w_2_tmp2 != NULL)
	{
	    err_check(ERROR_INVALID_ARG,"Aux mem must be all or none!");
	}
	w_2      = uquad_mat_alloc(LENGTH_INPUT,1);
	w_2_tmp  = uquad_mat_alloc(LENGTH_INPUT,1);
	w_2_tmp2 = uquad_mat_alloc(LENGTH_INPUT,1);
	local_mem = true;
	if((w_2 == NULL) || (w_2_tmp == NULL) || (w_2_tmp2 == NULL))
	{
	    cleanup_log_if(ERROR_MALLOC, "Failed to allocate mem for drag()!");
	}
    }
    retval = uquad_mat_dot_product(w_2,w,w);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(w_2_tmp, w_2, DRIVE_A1);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(w_2_tmp2, w, DRIVE_A2);
    err_propagate(retval);
    retval = uquad_mat_add(drive,w_2_tmp,w_2_tmp2);
    err_propagate(retval);
    cleanup:
    if(local_mem)
    {
	uquad_mat_free(w_2);
	uquad_mat_free(w_2_tmp);
	uquad_mat_free(w_2_tmp2);
    }
    return retval;
}

int f(struct uquad_mat* fx, kalman_io_t* kd)
{
    double x     = kd -> x_hat -> m_full[0];
    double y     = kd -> x_hat -> m_full[1];
    double z     = kd -> x_hat -> m_full[2];
    double psi   = kd -> x_hat -> m_full[3];
    double phi   = kd -> x_hat -> m_full[4];
    double theta = kd -> x_hat -> m_full[5];
    double vqx   = kd -> x_hat -> m_full[6];
    double vqy   = kd -> x_hat -> m_full[7];
    double vqz   = kd -> x_hat -> m_full[8];
    double wqx   = kd -> x_hat -> m_full[9];
    double wqy   = kd -> x_hat -> m_full[10];
    double wqz   = kd -> x_hat -> m_full[11];
    double abx   = 0;
    double aby   = 0;
    double abz   = 0;
    double T     = kd -> T;
    double weight= kd -> weight;

#if KALMAN_BIAS
    abx   = kd -> x_hat -> m_full[12];
    aby   = kd -> x_hat -> m_full[13];
    abz   = kd -> x_hat -> m_full[14];
#endif // KALMAN_BIAS

    int retval;
    double* w    = kd -> u -> m_full;
    struct uquad_mat* w_mat    = kd -> u;
    retval =  drive(TM,w_mat,w_2,w_2_tmp,w_2_tmp2);
    err_propagate(retval);
    retval =  drag(D,w_mat,w_2,w_2_tmp,w_2_tmp2);
    err_propagate(retval);
    double* TM_vec = TM->m_full;
    double* D_vec = D->m_full;
    fx->m_full[0]  = x     + T *(vqx*cos(phi)*cos(theta)+vqy*(cos(theta)*sin(phi)*sin(psi)-cos(phi)*sin(theta))+vqz*(sin(psi)*sin(theta)+cos(psi)*cos(theta)*sin(phi)) ) ;
    fx->m_full[1]  = y     + T *(vqx*cos(phi)*sin(theta)+vqy*(sin(theta)*sin(phi)*sin(psi)+cos(psi)*cos(theta))+vqz*(cos(psi)*sin(theta)*sin(phi)-cos(theta)*sin(psi)) ) ;
    fx->m_full[2]  = z     + T *(-vqx*sin(phi)+vqy*cos(phi)*sin(psi)+vqz*cos(psi)*cos(psi));
    fx->m_full[3]  = psi   + T*( wqx+wqz*tan(phi)*cos(psi)+wqy*tan(phi)*sin(psi));
    fx->m_full[4]  = phi   + T*( wqy*cos(psi)-wqz*sin(psi));
    fx->m_full[5]  = theta + T*( wqz*cos(psi)/cos(phi)+wqy*sin(psi)/cos(phi));
    fx->m_full[6]  = vqx   + T*( vqy*wqz-vqz*wqy+GRAVITY*sin(phi)+abx);
    fx->m_full[7]  = vqy   + T*( vqz*wqx-vqx*wqz-GRAVITY*cos(phi)*sin(psi)+aby);
    fx->m_full[8]  = vqz   + T*( vqx*wqy-vqy*wqx-GRAVITY*cos(phi)*cos(psi)+1/weight*(TM_vec[0]+TM_vec[1]+TM_vec[2]+TM_vec[3])+abz);
    fx->m_full[9] = wqx   + T*( wqy*wqz*(IYY-IZZ)+wqy*IZZM*(w[0]-w[1]+w[2]-w[3])+LENGTH*(TM_vec[1]-TM_vec[3]) )/IXX ;
    fx->m_full[10] = wqy   + T*( wqx*wqz*(IZZ-IXX)+wqx*IZZM*(w[0]-w[1]+w[2]-w[3])+LENGTH*(TM_vec[2]-TM_vec[0]) )/IYY;
    // fx->m_full[11] = wqz   + T*( -IZZM*(dw[0]-dw[1]+dw[2]-dw[3])+D[0]-D[1]+D[2]-D[3] )/IZZ;
    fx->m_full[11] = wqz   - T*( D_vec[0]-D_vec[1]+D_vec[2]-D_vec[3] )/IZZ;
#if KALMAN_BIAS
    fx->m_full[12] = abx;
    fx->m_full[13] = aby;
    fx->m_full[14] = abz;
#endif // KALMAN_BIAS

    return ERROR_OK;
}

int h(struct uquad_mat* hx, kalman_io_t* kd/*, uquad_bool_t is_gps*/)
{
    int retval;
    if(TM == NULL)
    {
	TM = uquad_mat_alloc(4,1); // TODO verificar que hay memoria y se ejecuto la sentencia correctamente
	D = uquad_mat_alloc(4,1);
    }
    retval = drive(TM,kd->u,w_2,w_2_tmp,w_2_tmp2);
    err_propagate(retval);
    double* TM_vec = TM -> m_full;
    double
	abx = 0,
	aby = 0,
	abz = 0;
#if KALMAN_BIAS
    abx = kd -> x_ -> m_full[SV_BAX];
    aby = kd -> x_ -> m_full[SV_BAY];
    abz = kd -> x_ -> m_full[SV_BAZ];
#endif
    hx->m_full[0]  = kd -> x_ -> m_full[SV_PSI];
    hx->m_full[1]  = kd -> x_ -> m_full[SV_PHI];
    hx->m_full[2]  = kd -> x_ -> m_full[SV_THETA];
    hx->m_full[3]  = abx;
    hx->m_full[4]  = aby;
    hx->m_full[5]  = 1/kd->weight*(TM_vec[0]+TM_vec[1]+TM_vec[2]+TM_vec[3]) + abz;
    hx->m_full[6]  = kd -> x_ -> m_full[SV_WQX];
    hx->m_full[7]  = kd -> x_ -> m_full[SV_WQY];
    hx->m_full[8]  = kd -> x_ -> m_full[SV_WQZ];
    hx->m_full[9]  = kd -> x_ -> m_full[SV_Z];
/*    if(is_gps)*/
/*    {*/
/*	hx->m_full[9] = kd -> x_ -> m_full[SV_X];*/
/*	hx->m_full[10] = kd -> x_ -> m_full[SV_Y];*/
/*	hx->m_full[11] = kd -> x_ -> m_full[SV_Z];*/
/*    }*/
    return ERROR_OK; 
}

int FF(uquad_mat_t* Fx, kalman_io_t* kd)
{
    // unused vars
    //    double x     = kd -> x_hat -> m_full[0];
    //    double y     = kd -> x_hat -> m_full[1];
    //    double z     = kd -> x_hat -> m_full[2];
    double psi   = kd -> x_hat -> m_full[3];
    double phi   = kd -> x_hat -> m_full[4];
    double theta = kd -> x_hat -> m_full[5];
    double vqx   = kd -> x_hat -> m_full[6];
    double vqy   = kd -> x_hat -> m_full[7];
    double vqz   = kd -> x_hat -> m_full[8];
    double wqx   = kd -> x_hat -> m_full[9];
    double wqy   = kd -> x_hat -> m_full[10];
    double wqz   = kd -> x_hat -> m_full[11];
    double T     = kd -> T;

    double* w    = kd -> u -> m_full;
    struct uquad_mat* w_t    = kd -> u;
    int retval;

    retval =  drive(TM,w_t,w_2,w_2_tmp,w_2_tmp2);
    err_propagate(retval);
    retval =  drag(D,w_t,w_2,w_2_tmp,w_2_tmp2);
    err_propagate(retval);

    Fx->m[0][0] = 1;
    Fx->m[0][1] = 0;
    Fx->m[0][2] = 0;
    Fx->m[0][3] = T*(vqz*(cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi)) + vqy*cos(psi)*cos(theta)*sin(phi));
    Fx->m[0][4] = T*(vqy*(sin(phi)*sin(theta) + cos(phi)*cos(theta)*sin(psi)) - vqx*cos(theta)*sin(phi) + vqz*cos(phi)*cos(psi)*cos(theta));
    Fx->m[0][5] = -T*(vqy*(cos(phi)*cos(theta) + sin(phi)*sin(psi)*sin(theta)) - vqz*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + vqx*cos(phi)*sin(theta));
    Fx->m[0][6] = T*cos(phi)*cos(theta);
    Fx->m[0][7] = -T*(cos(phi)*sin(theta) - cos(theta)*sin(phi)*sin(psi));
    Fx->m[0][8] = T*(sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi));
    Fx->m[0][9] = 0;
    Fx->m[0][10] = 0;
    Fx->m[0][11] = 0;

    Fx->m[1][0] = 0;
    Fx->m[1][1] = 1;
    Fx->m[1][2] = 0;
    Fx->m[1][3] = -T*(vqy*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + vqz*(cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta)));
    Fx->m[1][4] = T*(vqz*cos(phi)*cos(psi)*sin(theta) - vqx*sin(phi)*sin(theta) + vqy*cos(phi)*sin(psi)*sin(theta));
    Fx->m[1][5] = T*(vqz*(sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi)) - vqy*(cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi)) + vqx*cos(phi)*cos(theta));
    Fx->m[1][6] = T*cos(phi)*sin(theta);
    Fx->m[1][7] = T*(cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta));
    Fx->m[1][8] = -T*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta));
    Fx->m[1][9] = 0;
    Fx->m[1][10] = 0;
    Fx->m[1][11] = 0;

    Fx->m[2][0] = 0;
    Fx->m[2][1] = 0;
    Fx->m[2][2] = 1;
    Fx->m[2][3] = T*(vqy*cos(phi)*cos(psi) - 2*vqz*cos(psi)*sin(psi));
    Fx->m[2][4] = -T*(vqx*cos(phi) + vqy*sin(phi)*sin(psi));
    Fx->m[2][5] = 0;
    Fx->m[2][6] = -T*sin(phi);
    Fx->m[2][7] = T*cos(phi)*sin(psi);
    Fx->m[2][8] = T*uquad_square(cos(psi));
    Fx->m[2][9] = 0;
    Fx->m[2][10] = 0;
    Fx->m[2][11] = 0;

    Fx->m[3][0] = 0;
    Fx->m[3][1] = 0;
    Fx->m[3][2] = 0;
    Fx->m[3][3] = T*(wqy*cos(psi)*tan(phi) - wqz*sin(psi)*tan(phi)) + 1;
    Fx->m[3][4] = T*(wqz*cos(psi)*(uquad_square(tan(phi)) + 1) + wqy*sin(psi)*(uquad_square(tan(phi)) + 1));
    Fx->m[3][5] = 0;
    Fx->m[3][6] = 0;
    Fx->m[3][7] = 0;
    Fx->m[3][8] = 0;
    Fx->m[3][9] = T;
    Fx->m[3][10] = T*sin(psi)*tan(phi);
    Fx->m[3][11] = T*cos(psi)*tan(phi);

    Fx->m[4][0] = 0;
    Fx->m[4][1] = 0;
    Fx->m[4][2] = 0;
    Fx->m[4][3] = -T*(wqz*cos(psi) + wqy*sin(psi));
    Fx->m[4][4] = 1;
    Fx->m[4][5] = 0;
    Fx->m[4][6] = 0;
    Fx->m[4][7] = 0;
    Fx->m[4][8] = 0;
    Fx->m[4][9] = 0;
    Fx->m[4][10] = T*cos(psi);
    Fx->m[4][11] = -T*sin(psi);

    Fx->m[5][0] = 0;
    Fx->m[5][1] = 0;
    Fx->m[5][2] = 0;
    Fx->m[5][3] = T*((wqy*cos(psi))/cos(phi) - (wqz*sin(psi))/cos(phi));
    Fx->m[5][4] = T*((wqz*cos(psi)*sin(phi))/uquad_square(cos(phi)) + (wqy*sin(phi)*sin(psi))/uquad_square(cos(phi)));
    Fx->m[5][5] = 1;
    Fx->m[5][6] = 0;
    Fx->m[5][7] = 0;
    Fx->m[5][8] = 0;
    Fx->m[5][9] = 0;
    Fx->m[5][10] = (T*sin(psi))/cos(phi);
    Fx->m[5][11] = (T*cos(psi))/cos(phi);

    Fx->m[6][0] = 0;
    Fx->m[6][1] = 0;
    Fx->m[6][2] = 0;
    Fx->m[6][3] = 0;
    Fx->m[6][4] = T*GRAVITY*cos(phi);
    Fx->m[6][5] = 0;
    Fx->m[6][6] = 1;
    Fx->m[6][7] = T*wqz;
    Fx->m[6][8] = -T*wqy;
    Fx->m[6][9] = 0;
    Fx->m[6][10] = -T*vqz;
    Fx->m[6][11] = T*vqy;

    Fx->m[7][0] = 0;
    Fx->m[7][1] = 0;
    Fx->m[7][2] = 0;
    Fx->m[7][3] = -T*GRAVITY*cos(phi)*cos(psi);
    Fx->m[7][4] = T*GRAVITY*sin(phi)*sin(psi);
    Fx->m[7][5] = 0;
    Fx->m[7][6] = -T*wqz;
    Fx->m[7][7] = 1;
    Fx->m[7][8] = T*wqx;
    Fx->m[7][9] = T*vqz;
    Fx->m[7][10] = 0;
    Fx->m[7][11] = -T*vqx;

    Fx->m[8][0] = 0;
    Fx->m[8][1] = 0;
    Fx->m[8][2] = 0;
    Fx->m[8][3] = T*GRAVITY*cos(phi)*sin(psi);
    Fx->m[8][4] = T*GRAVITY*cos(psi)*sin(phi);
    Fx->m[8][5] = 0;
    Fx->m[8][6] = T*wqy;
    Fx->m[8][7] = -T*wqx;
    Fx->m[8][8] = 1;
    Fx->m[8][9] = -T*vqy;
    Fx->m[8][10] = T*vqx;
    Fx->m[8][11] = 0;

    Fx->m[9][0] = 0;
    Fx->m[9][1] = 0;
    Fx->m[9][2] = 0;
    Fx->m[9][3] = 0;
    Fx->m[9][4] = 0;
    Fx->m[9][5] = 0;
    Fx->m[9][6] = 0;
    Fx->m[9][7] = 0;
    Fx->m[9][8] = 0;
    Fx->m[9][9] = 1;
    Fx->m[9][10] = (T*(wqz*(IYY - IZZ) + IZZM*(w[0] - w[1] + w[2] - w[3])))/IXX;
    Fx->m[9][11] = (T*wqy*(IYY - IZZ))/IXX;

    Fx->m[10][0] = 0;
    Fx->m[10][1] = 0;
    Fx->m[10][2] = 0;
    Fx->m[10][3] = 0;
    Fx->m[10][4] = 0;
    Fx->m[10][5] = 0;
    Fx->m[10][6] = 0;
    Fx->m[10][7] = 0;
    Fx->m[10][8] = 0;
    Fx->m[10][9] = -(T*(wqz*(IXX - IZZ) - IZZM*(w[0]-w[1]+w[2]-w[3])))/IYY;
    Fx->m[10][10] = 1;
    Fx->m[10][11] = -(T*wqx*(IXX - IZZ))/IYY;

    Fx->m[11][0] = 0;
    Fx->m[11][1] = 0;
    Fx->m[11][2] = 0;
    Fx->m[11][3] = 0;
    Fx->m[11][4] = 0;
    Fx->m[11][5] = 0;
    Fx->m[11][6] = 0;
    Fx->m[11][7] = 0;
    Fx->m[11][8] = 0;
    Fx->m[11][9] = 0;
    Fx->m[11][10] = 0;
    Fx->m[11][11] = 1;

#if KALMAN_BIAS
    Fx->m[0][12] = 0;
    Fx->m[0][13] = 0;
    Fx->m[0][14] = 0;

    Fx->m[1][12] = 0;
    Fx->m[1][13] = 0;
    Fx->m[1][14] = 0;

    Fx->m[2][12] = 0;
    Fx->m[2][13] = 0;
    Fx->m[2][14] = 0;

    Fx->m[3][12] = 0;
    Fx->m[3][13] = 0;
    Fx->m[3][14] = 0;

    Fx->m[4][12] = 0;
    Fx->m[4][13] = 0;
    Fx->m[4][14] = 0;

    Fx->m[5][12] = 0;
    Fx->m[5][13] = 0;
    Fx->m[5][14] = 0;

    Fx->m[6][12] = T;
    Fx->m[6][13] = 0;
    Fx->m[6][14] = 0;

    Fx->m[7][12] = 0;
    Fx->m[7][13] = T;
    Fx->m[7][14] = 0;

    Fx->m[8][12] = 0;
    Fx->m[8][13] = 0;
    Fx->m[8][14] = T;

    Fx->m[9][12] = 0;
    Fx->m[9][13] = 0;
    Fx->m[9][14] = 0;

    Fx->m[10][12] = 0;
    Fx->m[10][13] = 0;
    Fx->m[10][14] = 0;

    Fx->m[11][12] = 0;
    Fx->m[11][13] = 0;
    Fx->m[11][14] = 0;

    Fx->m[12][0] = 0;
    Fx->m[12][1] = 0;
    Fx->m[12][2] = 0;
    Fx->m[12][3] = 0;
    Fx->m[12][4] = 0;
    Fx->m[12][5] = 0;
    Fx->m[12][6] = 0;
    Fx->m[12][7] = 0;
    Fx->m[12][8] = 0;
    Fx->m[12][9] = 0;
    Fx->m[12][10] = 0;
    Fx->m[12][11] = 0;
    Fx->m[12][12] = 1;
    Fx->m[12][13] = 0;
    Fx->m[12][14] = 0;

    Fx->m[13][0] = 0;
    Fx->m[13][1] = 0;
    Fx->m[13][2] = 0;
    Fx->m[13][3] = 0;
    Fx->m[13][4] = 0;
    Fx->m[13][5] = 0;
    Fx->m[13][6] = 0;
    Fx->m[13][7] = 0;
    Fx->m[13][8] = 0;
    Fx->m[13][9] = 0;
    Fx->m[13][10] = 0;
    Fx->m[13][11] = 0;
    Fx->m[13][12] = 0;
    Fx->m[13][13] = 1;
    Fx->m[13][14] = 0;

    Fx->m[14][0] = 0;
    Fx->m[14][1] = 0;
    Fx->m[14][2] = 0;
    Fx->m[14][3] = 0;
    Fx->m[14][4] = 0;
    Fx->m[14][5] = 0;
    Fx->m[14][6] = 0;
    Fx->m[14][7] = 0;
    Fx->m[14][8] = 0;
    Fx->m[14][9] = 0;
    Fx->m[14][10] = 0;
    Fx->m[14][11] = 0;
    Fx->m[14][12] = 0;
    Fx->m[14][13] = 0;
    Fx->m[14][14] = 1;
#endif // KALMAN_BIAS

    return ERROR_OK;
}

/*void uquad_kalman_inertial_aux_mem_deinit(void)*/
/*{*/
/*    uquad_mat_free(Fk_1_inertial);*/
/*    uquad_mat_free(Fk_1_T_inertial);*/
/*    uquad_mat_free(mtmp_inertial);*/
/*    uquad_mat_free(fx_inertial);*/
/*    uquad_mat_free(hx_inertial);*/
/*    uquad_mat_free(yk_inertial);*/
/*    uquad_mat_free(H_inertial);*/
/*    uquad_mat_free(HT_inertial);*/
/*    uquad_mat_free(HP__inertial);*/
/*    uquad_mat_free(HP_HT_inertial);*/
/*    uquad_mat_free(Sk_inertial);*/
/*    uquad_mat_free(Sk_1_inertial);*/
/*    uquad_mat_free(Sk_aux1_inertial);*/
/*    uquad_mat_free(Sk_aux2_inertial);*/
/*    uquad_mat_free(P_HT_inertial);*/
/*    uquad_mat_free(Kk_inertial);*/
/*    uquad_mat_free(Kkyk_inertial);*/
/*    uquad_mat_free(KkH_inertial);*/
/*    uquad_mat_free(IKH_inertial);*/
/*    uquad_mat_free(I_inertial);*/
/*}*/

int uquad_kalman_inertial_aux_mem_init(void)
{
    int retval;
    if(Fk_1_inertial != NULL)
    {
	err_check(ERROR_FAIL,"Memory has already been allocated!");
    }
    Fk_1_inertial   = uquad_mat_alloc(STATE_COUNT+STATE_BIAS,STATE_COUNT+STATE_BIAS);
    mem_alloc_check_ret_err(Fk_1_inertial);
    Fk_1_T_inertial = uquad_mat_alloc(STATE_COUNT+STATE_BIAS,STATE_COUNT+STATE_BIAS);
    mem_alloc_check_ret_err(Fk_1_T_inertial);
    mtmp_inertial   = uquad_mat_alloc(STATE_COUNT+STATE_BIAS,STATE_COUNT+STATE_BIAS);
    mem_alloc_check_ret_err(mtmp_inertial);
    retval = H_init(&H_inertial, false);
    err_propagate(retval);
    fx_inertial     = uquad_mat_alloc(STATE_COUNT+STATE_BIAS+3,1);
    mem_alloc_check_ret_err(fx_inertial);
    hx_inertial     = uquad_mat_alloc(10,1);
    mem_alloc_check_ret_err(hx_inertial);

    // Auxiliares para el update
    yk_inertial     = uquad_mat_alloc(hx_inertial->r,hx_inertial->c);
    mem_alloc_check_ret_err(yk_inertial);
    HT_inertial     = uquad_mat_alloc(H_inertial->c, H_inertial->r);
    mem_alloc_check_ret_err(HT_inertial);
    HP__inertial    = uquad_mat_alloc(H_inertial->r,STATE_COUNT + STATE_BIAS);
    mem_alloc_check_ret_err(HP__inertial);
    HP_HT_inertial  = uquad_mat_alloc(HP__inertial->r,HP__inertial->r);
    mem_alloc_check_ret_err(HP_HT_inertial);
    Sk_inertial     = uquad_mat_alloc(HP_HT_inertial->r,HP_HT_inertial->c);
    mem_alloc_check_ret_err(Sk_inertial);
    Sk_1_inertial   = uquad_mat_alloc(Sk_inertial->r,Sk_inertial->c);
    mem_alloc_check_ret_err(Sk_1_inertial);
    Sk_aux1_inertial = uquad_mat_alloc(Sk_inertial->r,Sk_inertial->c);
    mem_alloc_check_ret_err(Sk_aux1_inertial);
    Sk_aux2_inertial = uquad_mat_alloc(Sk_inertial->r,Sk_inertial->c << 1);
    mem_alloc_check_ret_err(Sk_aux2_inertial);
    P_HT_inertial   = uquad_mat_alloc(STATE_COUNT + STATE_BIAS, HT_inertial->c);
    mem_alloc_check_ret_err(P_HT_inertial);
    Kk_inertial     = uquad_mat_alloc(P_HT_inertial->r,Sk_1_inertial->c);
    mem_alloc_check_ret_err(Kk_inertial);
    Kkyk_inertial   = uquad_mat_alloc(Kk_inertial->r,yk_inertial->c);
    mem_alloc_check_ret_err(Kkyk_inertial);
    KkH_inertial    = uquad_mat_alloc(Kk_inertial->r, H_inertial->c);
    mem_alloc_check_ret_err(KkH_inertial);
    IKH_inertial    = uquad_mat_alloc(KkH_inertial->r, KkH_inertial->c);
    mem_alloc_check_ret_err(IKH_inertial);
    I_inertial      = uquad_mat_alloc(KkH_inertial->r, KkH_inertial->c);
    mem_alloc_check_ret_err(I_inertial);
    return retval;
}

/*void uquad_kalman_gps_aux_mem_deinit(void)*/
/*{*/
/*    uquad_mat_free(Fk_1_gps);*/
/*    uquad_mat_free(Fk_1_T_gps);*/
/*    uquad_mat_free(mtmp_gps);*/
/*    uquad_mat_free(fx_gps);*/
/*    uquad_mat_free(hx_gps);*/
/*    uquad_mat_free(yk_gps);*/
/*    uquad_mat_free(H_gps);*/
/*    uquad_mat_free(HT_gps);*/
/*    uquad_mat_free(HP__gps);*/
/*    uquad_mat_free(HP_HT_gps);*/
/*    uquad_mat_free(Sk_gps);*/
/*    uquad_mat_free(Sk_1_gps);*/
/*    uquad_mat_free(Sk_aux1_gps);*/
/*    uquad_mat_free(Sk_aux2_gps);*/
/*    uquad_mat_free(P_HT_gps);*/
/*    uquad_mat_free(Kk_gps);*/
/*    uquad_mat_free(Kkyk_gps);*/
/*    uquad_mat_free(KkH_gps);*/
/*    uquad_mat_free(IKH_gps);*/
/*    uquad_mat_free(I_gps);*/
/*}*/

int uquad_kalman_gps_aux_mem_init(void)
{
    int retval;
    if(Fk_1_gps != NULL)
    {
	err_check(ERROR_FAIL,"Memory has already been allocated!");
    }
    // Auxiliaries for prediction
    Fk_1_gps   = uquad_mat_alloc(STATE_COUNT + STATE_BIAS,STATE_COUNT + STATE_BIAS);
    mem_alloc_check_ret_err(Fk_1_gps);
    Fk_1_T_gps = uquad_mat_alloc(STATE_COUNT + STATE_BIAS,STATE_COUNT + STATE_BIAS);
    mem_alloc_check_ret_err(Fk_1_T_gps);
    mtmp_gps   = uquad_mat_alloc(STATE_COUNT + STATE_BIAS,STATE_COUNT + STATE_BIAS);
    mem_alloc_check_ret_err(mtmp_gps);
    retval = H_init(&H_gps, true);
    err_propagate(retval);
    // Auxiliaries for update
    fx_gps        = uquad_mat_alloc(STATE_COUNT + STATE_BIAS, 1);
    mem_alloc_check_ret_err(fx_gps);
    hx_gps        = uquad_mat_alloc(STATE_COUNT,1);
    mem_alloc_check_ret_err(hx_gps);
    // Auxiliares para el update
    yk_gps     = uquad_mat_alloc(hx_gps->r,hx_gps->c);
    mem_alloc_check_ret_err(yk_gps);
    HT_gps     = uquad_mat_alloc(H_gps->c, H_gps->r);
    mem_alloc_check_ret_err(HT_gps);
    HP__gps    = uquad_mat_alloc(H_gps->r,STATE_COUNT + STATE_BIAS);
    mem_alloc_check_ret_err(HP__gps);
    HP_HT_gps  = uquad_mat_alloc(HP__gps->r,HP__gps->r);
    mem_alloc_check_ret_err(HP_HT_gps);
    Sk_gps     = uquad_mat_alloc(HP_HT_gps->r,HP_HT_gps->c);
    mem_alloc_check_ret_err(Sk_gps);
    Sk_1_gps   = uquad_mat_alloc(Sk_gps->r,Sk_gps->c);
    mem_alloc_check_ret_err(Sk_1_gps);
    Sk_aux1_gps = uquad_mat_alloc(Sk_gps->r,Sk_gps->c);
    mem_alloc_check_ret_err(Sk_aux1_gps);
    Sk_aux2_gps = uquad_mat_alloc(Sk_gps->r,Sk_gps->c << 1);
    mem_alloc_check_ret_err(Sk_aux2_gps);
    P_HT_gps   = uquad_mat_alloc(STATE_COUNT + STATE_BIAS, HT_gps->c);
    mem_alloc_check_ret_err(P_HT_gps);
    Kk_gps     = uquad_mat_alloc(P_HT_gps->r,Sk_1_gps->c);
    mem_alloc_check_ret_err(Kk_gps);
    Kkyk_gps   = uquad_mat_alloc(Kk_gps->r,yk_gps->c);
    mem_alloc_check_ret_err(Kkyk_gps);
    KkH_gps    = uquad_mat_alloc(Kk_gps->r, H_gps->c);
    mem_alloc_check_ret_err(KkH_gps);
    IKH_gps    = uquad_mat_alloc(KkH_gps->r, KkH_gps->c);
    mem_alloc_check_ret_err(IKH_gps);
    I_gps      = uquad_mat_alloc(KkH_gps->r, KkH_gps->c);
    mem_alloc_check_ret_err(I_gps);
    return ERROR_OK;
}

/*void uquad_kalman_drive_drag_aux_mem_deinit(void)*/
/*{*/
/*    uquad_mat_free(w_2);*/
/*    uquad_mat_free(w_2_tmp);*/
/*    uquad_mat_free(w_2_tmp2);*/
/*    uquad_mat_free(TM);*/
/*    uquad_mat_free(D);*/
/*}*/

int uquad_kalman_drive_drag_aux_mem_init(void)
{
    w_2      = uquad_mat_alloc(LENGTH_INPUT,1);
    w_2_tmp  = uquad_mat_alloc(LENGTH_INPUT,1);
    w_2_tmp2 = uquad_mat_alloc(LENGTH_INPUT,1);
    TM       = uquad_mat_alloc(LENGTH_INPUT,1);
    D        = uquad_mat_alloc(LENGTH_INPUT,1);
    if(w_2 == NULL || w_2_tmp == NULL || w_2_tmp2 == NULL)
    {
	err_check(ERROR_MALLOC,"Failed to allocate memory!");
    }
    return ERROR_OK;
}

struct kalman_io_t* kalman_init()
{
    int retval;
    struct kalman_io_t* kd = (kalman_io_t*)malloc(sizeof(kalman_io_t));
    kd->x_hat = uquad_mat_alloc(STATE_COUNT+STATE_BIAS,1);
    kd->x_    = uquad_mat_alloc(STATE_COUNT+STATE_BIAS,1);
    kd->u     = uquad_mat_alloc(LENGTH_INPUT,1);
    kd->z     = uquad_mat_alloc(KALMAN_ROWS_H,1);
    kd->z_gps = uquad_mat_alloc(STATE_COUNT,1);
    kd->Q     = uquad_mat_alloc(STATE_COUNT+STATE_BIAS,STATE_COUNT+STATE_BIAS);
    kd->R     = uquad_mat_alloc(KALMAN_ROWS_H,KALMAN_ROWS_H);
    kd->P     = uquad_mat_alloc(STATE_COUNT+STATE_BIAS,STATE_COUNT+STATE_BIAS);
    kd->P_    = uquad_mat_alloc(STATE_COUNT+STATE_BIAS,STATE_COUNT+STATE_BIAS);
    kd->R_imu_gps = uquad_mat_alloc(STATE_COUNT,STATE_COUNT);

    retval = uquad_mat_zeros(kd->x_hat);
    cleanup_if(retval);
    retval = uquad_mat_zeros(kd->x_);
    cleanup_if(retval);
    retval = uquad_mat_zeros(kd->u);
    cleanup_if(retval);
    retval = uquad_mat_zeros(kd->Q);
    cleanup_if(retval);
    retval = uquad_mat_zeros(kd->R);
    cleanup_if(retval);
    retval = uquad_mat_zeros(kd->P);
    cleanup_if(retval);
    retval = uquad_mat_zeros(kd->R_imu_gps);
    cleanup_if(retval);
 
    kd->Q->m[0][0] = 100;
    kd->Q->m[1][1] = 100;
    kd->Q->m[2][2] = 100;
    kd->Q->m[3][3] = 1e-2;
    kd->Q->m[4][4] = 1e-2;
    kd->Q->m[5][5] = 1e-5;
    kd->Q->m[6][6] = 100;
    kd->Q->m[7][7] = 100;
    kd->Q->m[8][8] = 100;
    kd->Q->m[9][9] = 1e-1;
    kd->Q->m[10][10] = 1e-1;
    kd->Q->m[11][11] = 1e-1;
#if KALMAN_BIAS
    kd->Q->m[12][12] = 1;
    kd->Q->m[13][13] = 1;
    kd->Q->m[14][14] = 1;
#endif // KALMAN_BIAS

    kd->R->m[0][0] = COV_PSI_OK;  // psi 
    kd->R->m[1][1] = COV_PHI_OK;  // phi 
    kd->R->m[2][2] = COV_THE_OK;  // the 
    kd->R->m[3][3] = COV_ACC_OK;  // ax  
    kd->R->m[4][4] = COV_ACC_OK;  // ay  
    kd->R->m[5][5] = COV_ACC_OK;  // az  
    kd->R->m[6][6] = 1e-3;        // wqx 
    kd->R->m[7][7] = 1e-3;        // wqy 
    kd->R->m[8][8] = 1e-3;        // wqz 
    kd->R->m[9][9] = 1e5;         // z

    kd->R_imu_gps->m[0][0] = kd->R->m[0][0];
    kd->R_imu_gps->m[1][1] = kd->R->m[1][1];
    kd->R_imu_gps->m[2][2] = kd->R->m[2][2];
    kd->R_imu_gps->m[3][3] = kd->R->m[3][3];
    kd->R_imu_gps->m[4][4] = kd->R->m[4][4];
    kd->R_imu_gps->m[5][5] = kd->R->m[5][5];
    kd->R_imu_gps->m[6][6] = kd->R->m[6][6];
    kd->R_imu_gps->m[7][7] = kd->R->m[7][7];
    kd->R_imu_gps->m[8][8] = kd->R->m[8][8];
    kd->R_imu_gps->m[9][9] = 1e2;   // x 
    kd->R_imu_gps->m[10][10] = 1e2; // y 
    kd->R_imu_gps->m[11][11] = kd->R->m[9][9];

    kd->P->m[0][0] = 1;
    kd->P->m[1][1] = 1;
    kd->P->m[2][2] = 1;
    kd->P->m[3][3] = 1;
    kd->P->m[4][4] = 1;
    kd->P->m[5][5] = 1;
    kd->P->m[6][6] = 1;
    kd->P->m[7][7] = 1;
    kd->P->m[8][8] = 1;
    kd->P->m[9][9] = 1;
    kd->P->m[10][10] = 1;
    kd->P->m[11][11] = 1;
#if KALMAN_BIAS
    kd->P->m[12][12] = 1;
    kd->P->m[13][13] = 1;
    kd->P->m[14][14] = 1;
#endif // KALMAN_BIAS
    retval = uquad_mat_copy(kd->P_,kd->P);
    cleanup_if(retval);

    /// Initilization
    retval = uquad_mat_zeros(kd->x_hat);
    cleanup_if(retval);
    retval = uquad_mat_zeros(kd->x_);
    cleanup_if(retval);

    /// Aux memory for inertial kalman
    retval = uquad_kalman_inertial_aux_mem_init();
    cleanup_if(retval);

    /// Aux memory for gps kalman
    retval = uquad_kalman_gps_aux_mem_init();
    cleanup_if(retval);

    // Aux memory for drive/drag
    retval = uquad_kalman_drive_drag_aux_mem_init();
    cleanup_if(retval);

    return kd;
    cleanup:
/*    kalman_deinit(kd);*/
    return NULL;
}

int uquad_kalman(kalman_io_t * kd, /*struct uquad_mat* w,*/ imu_data_t* data, /*double T_us,*/ double weight/*, gps_comm_data_t *gps_i_data*/)
{
    int retval;
    double dtheta;
    uquad_bool_t is_gps = false;//(gps_i_data != NULL);
    struct uquad_mat
	*Fk_1    = (is_gps)?Fk_1_gps:Fk_1_inertial,
	*Fk_1_T  = (is_gps)?Fk_1_T_gps:Fk_1_T_inertial,
	*mtmp    = (is_gps)?mtmp_gps:mtmp_inertial,
	*H       = (is_gps)?H_gps:H_inertial,
	*hx      = (is_gps)?hx_gps:hx_inertial,
	*z       = (is_gps)?kd->z_gps:kd->z,
	*R       = (is_gps)?kd->R_imu_gps:kd->R,
	*P_      = kd->P_, /* ALWAYS */
	*P       = kd->P,  /* ALWAYS */
	*Q       = kd->Q,  /* ALWAYS */
	// Auxiliares Update
	*yk      = (is_gps)?yk_gps:yk_inertial,
	*HT      = (is_gps)?HT_gps:HT_inertial,
	*HP_     = (is_gps)?HP__gps:HP__inertial,
	*HP_HT   = (is_gps)?HP_HT_gps:HP_HT_inertial,
	*Sk      = (is_gps)?Sk_gps:Sk_inertial,
	*Sk_aux1 = (is_gps)?Sk_aux1_gps:Sk_aux1_inertial,
	*Sk_aux2 = (is_gps)?Sk_aux2_gps:Sk_aux2_inertial,
	*P_HT    = (is_gps)?P_HT_gps:P_HT_inertial,
	*Kk      = (is_gps)?Kk_gps:Kk_inertial,
	*Kkyk    = (is_gps)?Kkyk_gps:Kkyk_inertial,
	*KkH     = (is_gps)?KkH_gps:KkH_inertial,
	*IKH     = (is_gps)?IKH_gps:IKH_inertial,
	*Sk_1    = (is_gps)?Sk_1_gps:Sk_1_inertial,
	*I       = (is_gps)?I_gps:I_inertial;

    dtheta = data->magn->m_full[2] - kd->x_hat->m_full[SV_THETA];
    if (uquad_abs(dtheta) >= PI)
	data->magn->m_full[2] -= 2.0*PI*fix((dtheta+PI*sign(dtheta))/(2.0*PI));

/*    retval = store_data(kd, w, data, T_us, weight, gps_i_data);*/
    err_propagate(retval);

#if DYNAMIC_COV
    /// Dinamically adjust cov according to expected values
    if(data->acc_ok)
    {
	kd->R->m[0][0] = COV_PSI_OK; /* psi */
	kd->R->m[1][1] = COV_PHI_OK; /* phi */

	kd->R->m[3][3] = COV_ACC_OK; /* ax  */
	kd->R->m[4][4] = COV_ACC_OK; /* ay  */
	kd->R->m[5][5] = COV_ACC_OK; /* az  */
    }
    else
    {
	kd->R->m[0][0] = COV_PSI_BAD; /* psi */
	kd->R->m[1][1] = COV_PHI_BAD; /* phi */

	kd->R->m[3][3] = COV_ACC_BAD; /* ax  */
	kd->R->m[4][4] = COV_ACC_BAD; /* ay  */
	kd->R->m[5][5] = COV_ACC_BAD; /* az  */
    }

    if(data->magn_ok)
    {
	kd->R->m[2][2] = COV_THE_OK; /* the */
    }
    else
    {
	kd->R->m[2][2] = COV_THE_BAD; /* the */
    }
#endif // DYNAMIC_COV

    // Prediction
    retval = f(kd -> x_, kd);
    err_propagate(retval);
    retval = FF(Fk_1, kd);
    err_propagate(retval);
    Transpose_Matrix(Fk_1_T->m_full, Fk_1->m_full, Fk_1->r, Fk_1->c);      //    retval = struct uquad_matranspose(Fk_1_T, Fk_1);
//    err_propagate(retval);
    retval = uquad_mat_prod(mtmp, Fk_1, P);
    err_propagate(retval);
    retval = uquad_mat_prod(Fk_1,mtmp, Fk_1_T); // Aca lo vuelvo a guardar en Fk_1 para no hacer otra variable temporal
    err_propagate(retval);
    retval = uquad_mat_add(P_,Fk_1,Q);
    err_propagate(retval);

    // Update
    retval = h(hx, kd/*, is_gps**/);
    err_propagate(retval);
    retval =  uquad_mat_sub(yk, z , hx);
    err_propagate(retval);
    retval = uquad_mat_prod(HP_,H,P_);
    err_propagate(retval);
    Transpose_Matrix(Fk_1_T->m_full, Fk_1->m_full, Fk_1->r, Fk_1->c);      //    retval = struct uquad_matranspose(HT,H);
//    err_propagate(retval);
    retval = uquad_mat_prod(HP_HT,HP_,HT);
    err_propagate(retval);
    retval = uquad_mat_add(Sk,HP_HT,R); // Sk
    err_propagate(retval);
    retval = uquad_mat_inv(Sk_1,Sk,Sk_aux1,Sk_aux2);
    err_propagate(retval);
    retval = uquad_mat_prod(P_HT,P_,HT);
    err_propagate(retval);
    retval = uquad_mat_prod(Kk,P_HT,Sk_1);
    err_propagate(retval);
    retval = uquad_mat_prod(Kkyk,Kk,yk);
    err_propagate(retval);
    retval = uquad_mat_add(kd->x_hat, kd->x_, Kkyk);
    err_propagate(retval);
    retval =  uquad_mat_eye(I);
    err_propagate(retval);
    retval = uquad_mat_prod(KkH, Kk, H);
    err_propagate(retval);
    retval = uquad_mat_sub(IKH,I,KkH);
    err_propagate(retval);
    retval = uquad_mat_prod(P, IKH, P_);
    err_propagate(retval);

    return ERROR_OK;
}

/*void kalman_deinit(kalman_io_t *kd)*/
/*{*/
/*    /// Aux memory for inertial kalman*/
/*    uquad_kalman_inertial_aux_mem_deinit();*/

/*    /// Aux memory for gps kalman*/
/*    uquad_kalman_gps_aux_mem_deinit();*/

/*    // Aux memory for drive/drag*/
/*    uquad_kalman_drive_drag_aux_mem_deinit();*/

/*    if(kd != NULL)*/
/*    {*/
/*	uquad_mat_free(kd->x_hat);*/
/*	uquad_mat_free(kd->x_);*/
/*	uquad_mat_free(kd->u);*/
/*	uquad_mat_free(kd->z);*/
/*	uquad_mat_free(kd->z_gps);*/
/*	uquad_mat_free(kd->Q);*/
/*	uquad_mat_free(kd->R);*/
/*	uquad_mat_free(kd->P);*/
/*	uquad_mat_free(kd->P_);*/
/*	uquad_mat_free(kd->R_imu_gps);*/

/*	free(kd);*/
/*    }*/
/*}*/

/*int kalman_dump(kalman_io_t *kd, FILE *output)*/
/*{*/
/*    if(kd == NULL)*/
/*    {*/
/*	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid argument!");*/
/*    }*/
/*    log_msg(output,"Kalman - R");*/
/*    uquad_mat_dump(kd->R, output);*/
/*    log_msg(output,"Kalman - Q");*/
/*    uquad_mat_dump(kd->Q, output);*/
/*    log_msg(output,"Kalman - R_imu_gps");*/
/*    uquad_mat_dump(kd->R_imu_gps, output);*/
/*    return ERROR_OK;*/
/*}*/
