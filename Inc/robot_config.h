/*
 * robot_config.h
 *
 *  Created on: 10 nov 2026
 *      Author: uliss
 */

#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

#define PERIODO_MS 25

/*Conversiones de unidades*/
#define DEG_TO_RAD 0.01745329252f
#define RAD_TO_DEG 57.2957795f

/*Periodos de muestreo*/
#define TS_PD ((float)PERIODO_MS * 0.001f) //0.025
#define TS_PI (2.0f * TS_PD) //PI corre cada dos ticks

/*Ganancias*/
#define KP_E (-60.0f)
#define KD_E (-10.0f)

#define KP_V (0.0f)//(-7.0f)
#define KI_V (0.0f)//(-5.5f)

/*Limites*/
#define TAU_LIM_TOTAL 9.0f

/*Referencias*/
#define VEL_MAX_REF 0.40f
#define ACEL_REF 0.20f

#endif /* ROBOT_CONFIG_H_ */
