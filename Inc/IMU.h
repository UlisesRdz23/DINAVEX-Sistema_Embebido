/*
 * IMU.h
 *
 *  Created on: 2 nov 2026
 *      Author: uliss
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include "I2C.h"

#define IMU_ADDR 0x68

void IMU_init(void);
uint8_t IMU_who_am_i(void);
void IMU_read(void);
void IMU_calibrate_gyro(uint16_t muestras);

float IMU_get_ax(void);
float IMU_get_ay(void);
float IMU_get_az(void);
float IMU_get_gx(void);
float IMU_get_gy(void);
float IMU_get_gz(void);
float IMU_get_temp(void);
float IMU_get_pitch(void);

int16_t IMU_get_az_raw(void);

#endif /* IMU_H_ */
