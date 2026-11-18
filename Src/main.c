#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f4xx.h"
#include "uart.h"
#include "I2C.h"
#include "IMU.h"
#include "systick.h"
#include "tim.h"
#include "KalmanFilter.h"
#include "ZLAC8015.h"
#include "adc.h"
#include "control.h"
#include "robot_config.h"
#include "leds_indicadores.h"

#define CICLO_US 16U //16MHz*
#define TIMEOUT_RESP_US 3500U //Tiempo maximo de una respuesta
#define SILENCIO_US 2000U //Tiempo de silencio necesario en el modbus

#define Q_ANGLE  0.001f
#define Q_GYRO   0.003f
#define R_ANGLE  0.5f
#define C_0      1.0f
#define MAX_RPM 90
#define MAX_TAU 9

#define MOTORES_HABILITADOS 0
#define ANGULO_MAX_DEG 15.0f
#define K_TAU_A_RPM 10.0f

#define ESTADO_PARO 0
#define ESTADO_ACTIVO 1

#define LINEA_MAX 128
#define TIMEPUT_MOV_TICKS 20
//volatile uint32_t reloj_cpu_hz = 0;
//volatile uint32_t fuente_sysclk = 0;
static char linea[LINEA_MAX]; //JSON armado
static uint8_t linea_idx = 0; //Contador de caracteres
static uint32_t tick_ultimo_mov = 0;
static bool en_movimiento = false; // true cuando mande adelante/atras
volatile uint8_t flag_telemetria = 0;
static uint8_t contador_tel = 0;

volatile uint8_t estado = ESTADO_PARO;

volatile uint8_t rx_modbus1[16];
volatile uint8_t rx_modbus2[16];
volatile uint8_t rx_contador = 0;
volatile uint8_t rx_completado = 0;
volatile uint32_t tiempo_ms = 0;
volatile uint8_t idx_rx = 0;
volatile uint8_t vel_es = 0;
volatile int16_t velocidad = 0;

volatile uint8_t rx_escritura[16];
volatile uint8_t *rx_destino = 0;
static uint32_t t_fin_bus = 0;

volatile int b_UART = 0;
volatile int modo = 0;
volatile int falla_mot = 0;

volatile uint8_t imu_id;
volatile uint32_t contador = 0;
volatile uint32_t muestras_perdidas = 0;
volatile uint32_t tiempo_max_us = 0;
volatile uint32_t tiempo_max_us1 = 0;

volatile uint32_t volt_PA0 = 0;
volatile uint32_t volt_PA1 = 0;
volatile float voltaje_bateria = 0;

volatile float ax, ay, az;
volatile float gx, gy, gz;
volatile float pitch = 0.0f;
volatile float pitch_kalman = 0.0f;
volatile float roll = 0.0f;

volatile float velocidad_lineal = 0.0f;
volatile bool velocidad_valida = false;
volatile float tau_pendiente = 0.0f;
volatile float vel1;
volatile float vel2;

int led_rojo = 0;

static uint32_t ultimo = 0;

int lectura_d = 0;

volatile char key;
char buf_rx[20];

static char tlm[300];

KalmanFilter filtroPitch;
Control controlRobot;

static void DWT_init(void){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void inicializar_uarts(void){
	uart2_rxtx_init();
	uart3_rxtx_init();
	uart_printf(2, "UARTs inicizalizados \r\n");
	uart3_rx_interrupt_init();
	uart_printf(2, "Interrupciones UARTs inicizalizados \r\n");
}

static void iwdg_init(void){
	//Congelar el watchdog cuando el depurador pause el programa
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

	IWDG->KR = 0x5555; //Permitir escribir PR y RLR
	IWDG->PR = 3; //Reloj LSI 32 khz
	IWDG->RLR = 1000; //1000 cuentas; 1000ms
	IWDG->KR = 0xCCCC; //Arrancar watchdog
}

static bool respuesta_valida(volatile uint8_t *buf, uint8_t id, uint8_t funcion){
	//Lectura  7 bytes , escritura 8 bytes
	uint8_t n = (funcion == 0x03) ? 7: 8;
	uint8_t trama[16];

	if(rx_contador !=n) return false;
	for(int i = 0; i < n; i++){
		trama[i] = buf[i];
	}
	if(trama[0] != id || trama[1] != funcion) return false;

	uint16_t crc= calculo_CRC(trama, n-2);

	return trama[n - 2] == (crc & 0xFF) && trama[n - 1] == (crc >> 8);
}


static bool modbus_transaccion(uint8_t id, uint8_t funcion, uint16_t reg, uint16_t dato, volatile uint8_t *buf){
	//Silencio desde la respuesta anterior
	while((DWT->CYCCNT - t_fin_bus) < SILENCIO_US * CICLO_US){}

	//Preparar la recepción antes de enviar
	rx_contador = 0;
	rx_completado = 0;
	rx_destino = buf;

	//Enviar las funciones
	if(funcion == 0x03){
		ZLAC8015_read(id, reg, dato);
	}else{
		ZLAC8015_write(id, reg, dato);
	}

	//Esperar respuesta
	uint32_t t0 = DWT->CYCCNT;
	while(!rx_completado && (DWT->CYCCNT - t0) < TIMEOUT_RESP_US * CICLO_US){}

	rx_destino = 0;
	t_fin_bus = DWT->CYCCNT;
	return rx_completado && respuesta_valida(buf, id, funcion);

}

void inicializar_drivers(void){
	while(!modbus_transaccion(1, 0x03, REG_VOLTAJE,  1, rx_modbus1)){
		uart_printf(2, "Esperando driver 1 \r\n");
		systickDelaysMS(50);
	}
	systickDelaysMS(10);
	modbus_transaccion(1, 0x06, REG_TARGET_SPEED, 0, rx_escritura);
	systickDelaysMS(10);
	ZLAC8015_Minit(1);

	while(!modbus_transaccion(2, 0x03, REG_VOLTAJE,  1, rx_modbus1)){
		uart_printf(2, "Esperando driver 2 \r\n");
		systickDelaysMS(50);
	}
	systickDelaysMS(10);
	modbus_transaccion(2, 0x06, REG_TARGET_SPEED, 0, rx_escritura);
	systickDelaysMS(10);
	ZLAC8015_Minit(2);
	leds_indicadores('v');
}

void inicializar_control(void){
	//Obtener angulo inicial
	IMU_read();
	pitch = IMU_get_pitch();
	// Inicializar filtro con angulo real inicial
	KalmanFilter_Init(&filtroPitch, pitch);
	pitch_kalman = filtroPitch.angle;
	uart_printf(2, "Kalman iniciado a 40 Hz\r\n");
	//Inicializar variables del controlador
	control_Init(&controlRobot);
}

void calibracion(void){
	imu_id = IMU_who_am_i();
	if(imu_id != 0x68){
		uart_printf(2, "Error en id de la IMU \r\n");
		while(1){}
	}
	uart_printf(2, "Calibrando gyro \r\n");
	IMU_calibrate_gyro(1000);
}

void prioridades_de_interrupcion(void){
	// Prioridades de interrupciones
	NVIC_SetPriority(USART3_IRQn, 0); // Modbus
	NVIC_SetPriority(TIM2_IRQn, 1);   // IMU y Kalman
}

void leer_drivers(void){
	bool ok1 = modbus_transaccion(1, 0x03, REG_VELOCIDAD, 1, rx_modbus1);
	bool ok2 = modbus_transaccion(2, 0x03, REG_VELOCIDAD, 1, rx_modbus2);

	if(ok1){
		/*Obtener velocidad*/
		vel1 = conv_a_RPM(rx_modbus1[3], rx_modbus1[4]); //Nota checar signo
	}

	if(ok2){
		/*Obtener velocidad*/
		vel2 = conv_a_RPM(rx_modbus2[3], rx_modbus2[4]); //Nota checar signo
	}

	if(ok1 && ok2){
		float rpm_prom = (vel1 + (-vel2)) / 2;
		velocidad_lineal = rpm_prom * (2.0f * 3.14159265f * (diametro_rueda/2) / 60.0f);
		velocidad_valida = true;
		if(falla_mot >= 3){
			leds_indicadores('v');
			inicializar_drivers();
			falla_mot = 0;
		}
	}else{
		velocidad_valida = false;
		falla_mot++;
	}
}

void escribir_drivers(float tau, float theta){
	if(tau > MAX_TAU) tau = MAX_TAU;
	if(tau < -MAX_TAU) tau = -MAX_TAU;

	float par_requerido = tau/2;

	int16_t tau2 = (int32_t)(875.9*par_requerido + 176.2);
	int16_t tau1 = (int32_t)(952.6*par_requerido + 9.8);

	//Seguridad del robot
	bool caido = fabsf(theta) > ANGULO_MAX_DEG * DEG_TO_RAD;

	//Si el robot esta fuera del angulo establecido
	if(caido || estado != ESTADO_ACTIVO){
		controlRobot.integral_v = 0.0f;
		controlRobot.u_PI = 0.0f;
	}

	if(MOTORES_HABILITADOS == 0 || caido || estado != ESTADO_ACTIVO || falla_mot >= 3){
		tau1 = 0;
		tau2 = 0;
		if(led_rojo == 0){
			leds_indicadores('r');
			leds_indicadores('v');
			led_rojo = 1;
		}
	}else{
		if(led_rojo == 1){
			leds_indicadores('r');
			leds_indicadores('v');
			led_rojo = 0;
			falla_mot = 0;
		}
	}

	modbus_transaccion(1, 0x06, REG_TARGET_TORQUE, (uint16_t)(tau1), rx_escritura);
	modbus_transaccion(2, 0x06, REG_TARGET_TORQUE, (uint16_t)(-tau2), rx_escritura);
}

float lectura_bateria(void){
	volt_PA1 = adc_leer_canal(ADC_CH0); // AMC_VOUTN
	volt_PA0 = adc_leer_canal(ADC_CH1); // AMC_VOUTP

	float v_entrada = ((float)volt_PA1 - (float)volt_PA0) * VREF_ADC / ADC_MAX;
	return  v_entrada * FACTOR_DIVISOR;
}

void robot_arrancar(void){
	__disable_irq();
	controlRobot.integral_v = 0.0f;
	controlRobot.u_PI = 0.0f;
	controlRobot.v_ref = 0.0f;
	controlRobot.v_objetivo = 0.0f;
	estado = ESTADO_ACTIVO;
	__enable_irq();
}

void robot_parar(void){
	__disable_irq();
	estado = ESTADO_PARO;
	controlRobot.v_objetivo = 0.0f;
	__enable_irq();
}

static bool leer_linea(void){
	int c;

	while((c = uart2_rx_leer()) != -1){
		if(c == '{'){
			linea_idx = 0;
		}

		if(c == '\n' || c == '\r'){
			if(linea_idx > 0){
				linea[linea_idx] = '\0';
				linea_idx = 0;
				return true;
			}
		}else if(linea_idx < LINEA_MAX - 1){
			linea[linea_idx] = (char)c;
			linea_idx++;
		}else{
			linea_idx = 0;
		}
	}
	return false;
}

static void procesar_comando(void){
	if(!leer_linea()){
		return; //No hay comando completo aun
	}
	if(strcmp(linea, "{\"cmd\":\"paro\"}") == 0){
		robot_parar();
	}else if(strcmp(linea, "{\"cmd\":\"arranque\"}") == 0){
		if(estado == ESTADO_PARO){
			robot_arrancar();
		}
	}
}

static void telemetria(void){
	if(!flag_telemetria){
		return;
	}

	//Si DMA esta enviando el mensaje anterior saltar
	if(dma1_stream6_ocupado()){
		return;
	}
	flag_telemetria = 0;

	//Copiar datos de la ISR
	float inc = filtroPitch.angle;
	float v_izq = vel1;
	float v_der = -vel2;
	float tau = controlRobot.tau_real;
	float kp_e = controlRobot.kp_e;
	float kd_e = controlRobot.kd_e;
	float kp_v = controlRobot.kp_v;
	float ki_v = controlRobot.ki_v;
	uint8_t est = estado;

	//Bateria
	voltaje_bateria = lectura_bateria();

	//Par por rueda
	float par = 0.0f;
	if(est == ESTADO_ACTIVO){
		par = tau / 2.0f;
	}

	//Armar JASON
	int n = snprintf(tlm, sizeof(tlm),
			"{\"inclinacion\":%.2f,\"error\":%.2f,\"vel_izq\":%.2f,\""
			"vel_der\":%.2f,\"par_izq\":%.2f,\"par_der\":%.2f,\"pd_kp\""
			":%.3f,\"pd_kd\":%.3f,\"pi_kp\":%.3f,\"pi_ki\":%.3f,\"estado\""
			":%u,\"bat_24v\":%.2f}\n",
			inc, -inc, v_izq, v_der, par, par, kp_e, kd_e, kp_v, ki_v,
			(unsigned)est, voltaje_bateria);

	//Verificacion de error
	if(n <= 0 || n >= (int)sizeof(tlm)){
		return;
	}

	//Envio por DMA
	dma1_stream6_enviar(tlm, (uint16_t)n);
}

int main(){
	DWT_init();
	leds_init();
	leds_indicadores('a');
	inicializar_uarts();
	adc1_init();
	I2C_init();
	IMU_init();
	uart_printf(2, "Antes de drivers");
	//Inicializar drivers
	inicializar_drivers();
	uart_printf(2, "despues de drivers");
	calibracion();
	secuencia_buzzer();
	//Prioridades de interrupción
	prioridades_de_interrupcion();
	//Inicializar control
	inicializar_control();
	//|iwdg_init();
	dma1_stream6_init();
	dma1_stream5_rx_init();
	robot_arrancar();
	//Inicializar temporizador
	tim2_200hz_interrupt_init();

	while(1){
		procesar_comando();
		telemetria();
	}
}


void USART3_IRQHandler(){
	static uint8_t longitud = 0;

	if(USART3->SR & SR_RNXE){
		uint8_t dato = (uint8_t)(USART3->DR & 0xFF);

		if(rx_destino != 0 && rx_completado == 0 && rx_contador < 16){
			rx_destino[rx_contador] = dato;
			rx_contador++;

			if(rx_contador >= 3){
				if(rx_destino[1] == 0x03){
					longitud = 5 + rx_destino[2];
				}else if(rx_destino[1] == 0x06){
					longitud = 8;
				}else{
					longitud = 5;
				}
				if(rx_contador == longitud){
					rx_completado = 1;
				}
			}
		}
	}
}

void TIM2_IRQHandler(void){
	if(TIM2->SR & SR_UIF){
		/*Clear update interrupt flag*/
		TIM2->SR &= ~SR_UIF;
		/*Do something*/
		tim2_ticks++;

		/*Leer datos de la IMU*/
		IMU_read();
		/*Obtener inclinación*/
		pitch = IMU_get_pitch();
		/*Obtener velocidad angular*/
		gy = IMU_get_gy();
		/*Filtro de kalman*/
		KalmanFilter_F(&filtroPitch, pitch, gy, TS_PD, Q_ANGLE, Q_GYRO, R_ANGLE, C_0);
		/*Covertir a radianes*/
		float theta = filtroPitch.angle * DEG_TO_RAD;
		float omega = filtroPitch.angle_dot * DEG_TO_RAD;

		/*Ejecutar controlador*/
		if(control_5ms(&controlRobot, theta, omega, velocidad_lineal, velocidad_valida)){
			/*Guardar par calculado*/
			tau_pendiente = controlRobot.tau_real;
		}else{
			tau_pendiente = 0.0f;
		}

		/*Escribir en drivers*/
		escribir_drivers(tau_pendiente, theta);

		/*Leer drivers con el tick de control*/
		if(controlRobot.contador_PI + 1>= 2){
			leer_drivers();
		}

		/*Bandera de telemetria*/
		contador_tel++;
		if(contador_tel >= 50){
			contador_tel = 0;
			flag_telemetria = 1;
		}

		IWDG->KR = 0xAAAA;   //Alimentar el watchdog
	}
}
