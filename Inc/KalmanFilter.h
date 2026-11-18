/*
 * KalmanFilter.h
 *
 *  Created on: 6 nov 2026
 *      Author: uliss
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include <math.h>

typedef struct{
	//Resultados del filtro
	float angle;
	float angle6;
	float angle_dot;

	//Variables internas
	float angle_err;
	float q_bias;

	float PDot[4];
	float P[2][2];

	float PCt_0, PCt_1;
	float E, K_0, K_1;
	float t_0, t_1;
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf, float angulo_inicial);
void KalmanFilter_Yiorderfilter(KalmanFilter *kf, float angle_m, float gyro_m, float dt, float K1);
void KalmanFilter_F(KalmanFilter *kf, float angle_m, float gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0);

#endif /* KALMANFILTER_H_ */
