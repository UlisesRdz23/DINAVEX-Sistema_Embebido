/*
 * control.h
 *
 *  Created on: 10 nov 2026
 *      Author: uliss
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

#define diametro_rueda 0.165f

/*Variables del controlador*/
typedef struct{
	/*Gananacias*/
	float kp_e;
	float kd_e;
	float kp_v;
	float ki_v;
	/*Referencias de velocidad*/
	float v_objetivo;
	float v_ref;
	/*Controladores*/
	float u_PD;
	float u_PI;
	/*Integral PI*/
	float integral_v;
	/*Par calculado*/
	float tau_cmd;
	float tau_real;
	/*Contador PI 20ms*/
	uint8_t contador_PI;
}Control;

/*Inicializar variables de control*/
void control_Init(Control *ctrl);

/*Ejecutar un ciclo de 5ms*/
bool control_5ms(Control *ctrl, float theta, float omega_p, float velocidad, bool velocidad_valida);

/*Limitar variable*/
float control_limite(float valor, float minimo, float maximo);

#endif /* CONTROL_H_ */
