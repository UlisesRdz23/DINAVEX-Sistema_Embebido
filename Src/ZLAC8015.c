/*
 * ZLAC8015.c
 *
 *  Created on: 29 oct 2026
 *      Author: uliss
 */

#include "ZLAC8015.h"
#include "systick.h"

uint8_t id_driver = 0x00;

uint8_t write_struct[8] = {
		0x00,
		0x06, //Function code
		0x00, //High 8 bits of register start address
		0x00, //Low 8 bits of register start address
		0x00, // High 8 bits of register number
		0x00, // Low 8 bits of register number
		0x00, // High 8 bits of CRC check
		0x00 // Low 8 bits of CRC check
};

uint8_t read_struct[8] = {
		0x00, //direccion del driver
		0x03, //Function code
		0x00, //High 8 bits of register start address
		0x00, //Low 8 bits of register start address
		0x00, // High 8 bits of register number
		0x00, // Low 8 bits of register number
		0x00, // High 8 bits of CRC check
		0x00 // Low 8 bits of CRC check
};

uint16_t calculo_CRC(uint8_t *datos, int n){
	uint16_t CRC_Mod = 0xFFFF;

	for(int i = 0; i < n; i++){
		CRC_Mod ^= datos[i];

		for(int j = 0; j < 8; j++){
			if(CRC_Mod & 1){
				CRC_Mod = (CRC_Mod >> 1) ^ 0xA001;
			}else{
				CRC_Mod >>= 1;
			}
		}
	}

	return CRC_Mod;
}

void ZLAC8015_Minit(int id){
	ZLAC8015_write(id, REG_OPERATING_MODE, 4);
	uart_printf(2, "Establecer modo par \r\n");
	systickDelaysMS(10);
	ZLAC8015_write(id, REG_TORQUE_SLOPE, 150);
	uart_printf(2, "Establecer acceleracion \r\n");
	systickDelaysMS(10);
	ZLAC8015_write(id, REG_CONTROL_WORD, 8);
	uart_printf(2, "Habilitar motor \r\n");
	systickDelaysMS(10);
}

void ZLAC8015_read(uint8_t id,uint16_t registro, int n_register){
	//Guardar dirección del driver
	read_struct[0] = id;

	read_struct[2] = registro >> 8;
	read_struct[3] = registro & 0xFF;

	read_struct[4] = n_register >> 8;
	read_struct[5] = n_register & 0xFF;

	uint16_t crc_m = calculo_CRC(read_struct, 6);
	read_struct[6] = crc_m & 0xFF;
	read_struct[7] = crc_m >> 8;

	for(int i = 0; i<8; i++){
		uart3_write(read_struct[i]);
	}
	/*for(int i = 0; i < 8; i++){
		conv_a_cadenaC(read_struct[i]);
	}
	uart_printf(2, "\r\n");*/
}

void ZLAC8015_write(uint8_t id, uint16_t registro, uint16_t information_data){
	//Guardar dirección del driver
	write_struct[0] = id;

	write_struct[2] = registro >> 8;
	write_struct[3] = registro & 0xFF;

	write_struct[4] = information_data >> 8;
	write_struct[5] = information_data & 0xFF;

	uint16_t crc_m = calculo_CRC(write_struct, 6);
	write_struct[6] = crc_m & 0xFF;
	write_struct[7] = crc_m >> 8;

	for(int i = 0; i<8; i++){
		uart3_write(write_struct[i]);
	}
	/*for(int i = 0; i < 8; i++){
		conv_a_cadenaC(write_struct[i]);
	}
	uart_printf(2, "\r\n");*/
}

void conv_a_cadenaC(uint8_t dato){
	char esc[8];
	snprintf(esc, sizeof(esc), "%02X ", dato);
	uart_printf(2, esc);
}

float conv_a_RPM(uint8_t HD, uint8_t HL){
	uint16_t dato = ((uint16_t)HD << 8) | HL;

	int16_t rpm = (int16_t)dato;

	float RPMs = (float)rpm * 0.1f;

	return RPMs;
}

float conv_a_voltaje(uint8_t HD, uint8_t HL){
	uint16_t dato = ((uint16_t)HD << 8) | HL;

	int16_t volt = (int16_t)dato;

	float voltaje = (float)volt * 0.01f;

	return voltaje;
}
