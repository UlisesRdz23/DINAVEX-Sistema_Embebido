/*
 * IMU_calibrate.h
 *
 *  Created on: 5 nov 2026
 *      Author: uliss
 */

#ifndef IMU_CALIBRATION_H_
#define IMU_CALIBRATION_H_

// Offset por eje
#define ACC_BIAS_X   0.05285f
#define ACC_BIAS_Y   0.02725f
#define ACC_BIAS_Z  -0.2563f

//Escala de los factores
#define ACC_SCALE_X  0.99655f
#define ACC_SCALE_Y  0.99755f
#define ACC_SCALE_Z  1.0218f

// Referencia angular del robot
#define ANGULO_CERO  0.0f


#endif /* IMU_CALIBRATION_H_ */
