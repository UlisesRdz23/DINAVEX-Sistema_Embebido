/*
 * I2C.h
 *
 *  Created on: 2 nov 2026
 *      Author: uliss
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

void I2C_init(void);

/*Leer un solo byte de un registro*/
void I2C1_byteRead(uint8_t dir_e, uint8_t dir_m, uint8_t *data);
/*Leer varios bytes consecutivos*/
void I2C1_burstRead(uint8_t dir_e, uint8_t dir_m, uint16_t n, uint8_t *data);
/*Escribir varios bytes consecutivos*/
void I2C1_burstWrite(uint8_t dir_e, uint8_t dir_m, uint16_t n, uint8_t *data);

#endif /* I2C_H_ */
