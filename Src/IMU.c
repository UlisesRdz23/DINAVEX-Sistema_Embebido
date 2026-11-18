#include <math.h>
#include "IMU.h"
#include "uart.h"
#include "IMU_calibration.h"

#define RAD_TO_DEG 57.2957795f

static uint8_t data[14];

static float gx_off = 0.0;
static float gy_off = 0.0;
static float gz_off = 0.0;

static const float accel = 16384.0f;
static const float gyro = 65.5f;
static const float temp1 = 340.0f;
static const float temp2 = 36.53f;

static int16_t BH_BL(int i){
	uint16_t valor;

	valor = ((uint16_t)data[i]<<8) | (uint16_t)data[i+1];
	return (int16_t)valor;
}

void IMU_init(void){
	uint8_t temp;

	/*PWR_MGHMT_1: Despertar IMU*/
	temp = 0x01;
	I2C1_burstWrite(IMU_ADDR, 0x6B, 1, &temp);

	/*Config DLPF*/
	temp = 0x04;
	I2C1_burstWrite(IMU_ADDR, 0x1A, 1, &temp);

	/*Config gyro +/- 500°*/
	temp = 0x08;
	I2C1_burstWrite(IMU_ADDR, 0x1B, 1, &temp);

	/*Config accel +/- 2g*/
	temp = 0x00;
	I2C1_burstWrite(IMU_ADDR, 0x1C, 1, &temp);

	/*SMPLRT_DIV: Configurar frecuencia */
	temp = 0x00;
	I2C1_burstWrite(IMU_ADDR, 0x19, 1, &temp); // 1 kHz / (1+0) = 1 kHz
}

uint8_t IMU_who_am_i(void){
	uint8_t id = 0;

	I2C1_byteRead(IMU_ADDR, 0x75, &id);

	/*if(id != 0x68){
		   uart_printf(2, "Error en id de la IMU \r\n");
		   while(1){}
	}*/

	return (uint8_t)id;
}

void IMU_read(void){
	I2C1_burstRead(IMU_ADDR, 0x3B, 14, data);
}

void IMU_calibrate_gyro(uint16_t muestras){
	int32_t x = 0, y = 0, z = 0;

	if(muestras == 0){
		return;
	}

	/*Descartar las primeras lecturas*/
	for(int i = 0; i < 50; i++){
		IMU_read();
	}

	/*Sumar valores crudos (LSB) en enteros: no se pierde precision*/
	for(uint16_t i = 0; i < muestras; i++){
		IMU_read();
		x += BH_BL(8);
		y += BH_BL(10);
		z += BH_BL(12);
	}

	/*Promedio crudo -> grados/s*/
		gx_off = ((float)x / muestras) / gyro;
		gy_off = ((float)y / muestras) / gyro;
		gz_off = ((float)z / muestras) / gyro;
}

/*Funciones para obtenicion de datos*/
float IMU_get_ax(void){
	float ax_o = (float)BH_BL(0) / accel;

	return (ax_o - ACC_BIAS_X)/ ACC_SCALE_X;
}

float IMU_get_ay(void){
	float ay_o = (float)BH_BL(2) / accel;

	return (ay_o - ACC_BIAS_Y)/ ACC_SCALE_Y;
}

float IMU_get_az(void){
	float az_o = (float)BH_BL(4) / accel;

	return (az_o - ACC_BIAS_Z)/ ACC_SCALE_Z;
}

float IMU_get_gx(void){
	return BH_BL(8) / gyro - gx_off;
}

float IMU_get_gy(void){
	return BH_BL(10) / gyro - gy_off;
}

float IMU_get_gz(void){
	return BH_BL(12) / gyro - gz_off;
}

float IMU_get_temp(void){
	return BH_BL(6) / temp1 + temp2;
}

int16_t IMU_get_az_raw(void){
    return BH_BL(4);
}

float IMU_get_pitch(void){
	float ax = IMU_get_ax();
	float ay = IMU_get_ay();
	float az = IMU_get_az();

	return atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
}
