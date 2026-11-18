/*
 * ZLAC8015.h
 *
 *  Created on: 29 oct 2026
 *      Author: uliss
 */

#ifndef ZLAC8015_H_
#define ZLAC8015_H_

#include "uart.h"
#include <stdint.h>
#include <stdio.h>

/*Direcciones de registros del driver*/
#define REG_VERSION 0x2025
#define REG_TEMPERATURE  0x2026
#define REG_ESTADO 0x2027
#define REG_HALL 0x2028
#define REG_VOLTAJE  0x2029
#define REG_POSICION 0x202A
#define REG_VELOCIDAD 0x202C
#define REG_TORQUE 0x202D
#define REG_ERROR 0x202E
#define REG_OPERATING_MODE 0x2032
#define REG_Stype_accet 0x2037
#define REG_Stype_decet 0x2038
#define REG_CONTROL_WORD 0x2031
#define REG_TARGET_TORQUE 0x2033
#define REG_TARGET_SPEED 0x203A
#define REG_TORQUE_SLOPE 0x203B

/*Valores para seleccionar modos*/
#define MODE_VELOCIDAD 3
#define MODE_PAR 4

/*Comandos para el registro de control*/
#define CMD_EMERGENCIA 0x05
#define CMD_CLEAR_FAULT 0x06
#define CMD_STOP 0x07
#define CMD_ENABLE 0x08

void ZLAC8015_read(uint8_t id, uint16_t registro, int n_register);
void ZLAC8015_write(uint8_t id, uint16_t registro, uint16_t information_data);
void ZLAC8015_Minit(int id);

uint16_t calculo_CRC(uint8_t *datos, int n);
float conv_a_RPM(uint8_t HD, uint8_t HL);
float conv_a_voltaje(uint8_t HD, uint8_t HL);

void conv_a_cadenaC(uint8_t dato);


#endif /* ZLAC8015_H_ */
