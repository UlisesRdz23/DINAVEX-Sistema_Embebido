/*
 * control.c
 *
 *  Created on: 10 nov 2026
 *      Author: uliss
 */



#include <math.h>
#include "control.h"
#include "robot_config.h"

/*Limitar los valores*/
float control_limite(float valor, float minimo, float maximo){
	/*Si se excede devuelve el limite maximo*/
	if(valor > maximo){
		return maximo;
	}
	/*Si esta por debajo devuelve el limite minimo*/
	if(valor < minimo){
		return minimo;
	}
	/*Dentro del limite*/
	return valor;
}

/*Inicializar en 0 las variables*/
void control_Init(Control *ctrl){
	/*Evitar tra*/
	if(ctrl == 0){
		return;
	}
	/*Inicializar miembros en 0*/
	*ctrl = (Control){0};

	/*Ganancias iniciales del robot*/
	ctrl->kp_e = KP_E;
	ctrl->kd_e = KD_E;
	ctrl->kp_v = KP_V;
	ctrl->ki_v = KI_V;
}

/*Ejecutar control cada 5 ms*/
bool control_5ms(Control *ctrl, float theta, float omega_p, float velocidad, bool velocidad_valida){
	/*Verificar la estructura*/
	if(ctrl == 0){
		return false;
	}

	/*Verificar tamaños*/
	if(!isfinite(theta) || !isfinite(omega_p)){
		return false;
	}

	/*Verificar velocidad*/
	if(velocidad_valida && !isfinite(velocidad)){
		return false;
	}

	/***Rampa de referencia***/

	/*Cambio cada 5ms*/
	float paso_max = ACEL_REF * TS_PD;
	/*Diferencia entre el objetivo y la referencia*/
	float diferencia = ctrl->v_objetivo - ctrl->v_ref;
	/*limitar cambio de velocidad*/
	float paso = control_limite(diferencia, -paso_max, paso_max);
	/*Actualizar referencia interna*/
	ctrl->v_ref += paso;

	/***Control PD de equilibrio***/

	/*Referencia angular inicial = 0*/
	float e_theta = -theta;
	/*Referencia de velocidad angular = 0*/
	float e_omega = -omega_p;
	/*Ley PD*/
	ctrl->u_PD = ctrl->kp_e * e_theta + ctrl->kd_e * e_omega;

	/***Control PI de velocidad***/

	/*Incrementar contador PI*/
	ctrl->contador_PI++;

	if(ctrl->contador_PI >= 2){
		//Reiniciar contador
		ctrl->contador_PI = 0;

		// Si todavía no existe velocidad válida
		if(!velocidad_valida){
			//No acumular error
			ctrl->integral_v = 0.0f;
			//Desactivar contribucion PI
			ctrl->u_PI = 0.0f;
		}else{
			//Error de velocidad
			float e_v = ctrl->v_ref - velocidad;
			//Integral candidata
			float integral_candidata = ctrl->integral_v + TS_PI * e_v;
			//Salida con integral anterior
			float pi_actual = ctrl->kp_v * e_v + ctrl->ki_v * ctrl->integral_v;
			//Salida con integral candidata
			float pi_candidato = ctrl->kp_v * e_v + ctrl->ki_v * integral_candidata;
			//Par candidato total
			float tau_candidato = ctrl->u_PD + pi_candidato;
			//Cambio debido a la integral
			float delta_integral = pi_candidato - pi_actual;
			//Detectar si la integral empeora la saturación
			bool empeora = (tau_candidato > TAU_LIM_TOTAL && delta_integral > 0.0f) || (tau_candidato < -TAU_LIM_TOTAL && delta_integral < 0.0f);
			//Actualizar integral si es posible
			if(!empeora){
				ctrl->integral_v = integral_candidata;
			}
			//Calcular PI final
			ctrl->u_PI = ctrl->kp_v * e_v + ctrl->ki_v * ctrl->integral_v;
		}
	}

	/***Sumar PD + PI***/
	ctrl->tau_cmd = ctrl->u_PD + ctrl->u_PI;

	/***Saturación del par***/
	ctrl->tau_real = control_limite(ctrl->tau_cmd, -TAU_LIM_TOTAL, TAU_LIM_TOTAL);
	//Comprobar resultado numérico
	if(!isfinite(ctrl->tau_real)){
		return false;
	}

	//Calculo completado
	return true;
}
