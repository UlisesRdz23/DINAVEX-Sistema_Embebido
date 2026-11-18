#include "KalmanFilter.h"

void KalmanFilter_Init(KalmanFilter *kf, float angulo_inicial){
	/*Inicializar variables*/
	*kf = (KalmanFilter){0};

	/*Angulo inicial obtenido*/
	kf->angle = angulo_inicial;
	kf->angle6 = angulo_inicial;

	/*Mstriz de varianza inicial*/
	kf->P[0][0] = 1.0f;
	kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f;
	kf->P[1][1] = 1.0f;
}

void KalmanFilter_Yiorderfilter(KalmanFilter *kf, float angle_m, float gyro_m, float dt, float K1){
	kf->angle6 = K1*angle_m + (1.0f - K1) * (kf->angle6 + gyro_m * dt);
}

void KalmanFilter_F(KalmanFilter *kf, float angle_m, float gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0){
	if(dt<=0.0f || !isfinite(angle_m) || !isfinite(gyro_m)){
		return;
	}
	/*Predicción del angulo*/
	kf->angle += (gyro_m - kf->q_bias) * dt;

	/*Error entre medicion y predicción*/
	kf->angle_err = angle_m - kf->angle;

	/*Evolución de la convarianza*/
	kf->PDot[0] = Q_angle - kf->P[0][1] - kf->P[1][0];
	kf->PDot[1] = -kf->P[1][1];
	kf->PDot[2] = -kf->P[1][1];
	kf->PDot[3] = Q_gyro;

	kf->P[0][0] += kf->PDot[0] *dt;
	kf->P[0][1] += kf->PDot[1] *dt;
	kf->P[1][0] += kf->PDot[2] *dt;
	kf->P[1][1] += kf->PDot[3] *dt;

	/*Gananacia kalman*/
	kf->PCt_0 = C_0 * kf->P[0][0];
	kf->PCt_1 = C_0 * kf->P[1][0];

	kf->E = R_angle+ C_0 * kf->PCt_0;

	kf->K_0 = kf->PCt_0 /kf->E;
	kf->K_1 = kf->PCt_1 /kf->E;

	kf->t_0 = kf->PCt_0;
	kf->t_1 = C_0 * kf->P[0][1];

	/*Actualización de covarianza*/
	kf->P[0][0] -= kf->K_0 * kf->t_0;
	kf->P[0][1] -= kf->K_0 * kf->t_1;
	kf->P[1][0] -= kf->K_1 * kf->t_0;
	kf->P[1][1] -= kf->K_1 * kf->t_1;

	/*Correción del angulo y bias*/
	kf->angle += kf->K_0 * kf->angle_err;
	kf->q_bias += kf->K_1 * kf->angle_err;

	/*Velocidad angular corregida*/
	kf->angle_dot = gyro_m - kf->q_bias;
}
