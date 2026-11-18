#include "I2C.h"

#define GPIOBEN					 (1U<<1)
#define I2C1EN				   	 (1U<<21)

#define I2C_100khz				 80 //0b 0101 0000 decimal 80
#define SD_MODE_MAX_RISE_TIME	 17
#define CR1_PE					(1U<<0)

#define SR2_BUSY 				(1U<<1)
#define CR1_START 				(1U<<8)
#define SR1_SB 					(1U<<0)
#define SR1_ADDR 				(1U<<1)
#define SR1_TXE 				(1U<<7)
#define CR1_ACK 				(1U<<10)
#define CR1_STOP				(1U<<9)
#define SR1_RXE 				(1U<<6)
#define SR1_BTF 				(1U<<2)
#define CR1_POS 				(1U<<11)

/*PB8 --- SCL
 *PB9 --- SDA
 * */

void I2C_init(void){
	/*Enable clock acces to GPIOB*/
	RCC-> AHB1ENR |= GPIOBEN;

	/*Set PB8 and PB9 mode to alternate function*/
	GPIOB->MODER &= ~(1U<<16);
	GPIOB->MODER |= (1U<<17);

	GPIOB->MODER &= ~(1U<<18);
	GPIOB->MODER |= (1U<<19);

	/*Set PB8 and PB9 alterntive function type to I2C1-SCL and I2C1-SDA (AF04)*/
	GPIOB ->AFR[1] &= ~(1U<<0);
	GPIOB ->AFR[1] &= ~(1U<<1);
	GPIOB ->AFR[1] |= (1U<<2);
	GPIOB ->AFR[1] &= ~(1U<<3);

	GPIOB ->AFR[1] &= ~(1U<<4);
	GPIOB ->AFR[1] &= ~(1U<<5);
	GPIOB ->AFR[1] |= (1U<<6);
	GPIOB ->AFR[1] &= ~(1U<<7);

	/*Set PB8 and PB9 output type to open drain*/
	GPIOB->OTYPER |= (1U<<8);
	GPIOB->OTYPER |= (1U<<9);

	/*Enable Pullup for PB8 and PB9*/
	GPIOB->PUPDR |= (1U<<16);
	GPIOB->PUPDR &= ~(1U<<17);

	GPIOB->PUPDR |= (1U<<18);
	GPIOB->PUPDR &= ~(1U<<19);

	/*Enable clock acces to I2C*/
	RCC->APB1ENR |= I2C1EN;

	/*Enter reset mode*/
	I2C1->CR1 |= (1U<<15);
	/*Come out of reset mode*/
	I2C1->CR1 &= ~(1U<<15);

	/*Set Peripheral clock frequency*/
	I2C1->CR2 = (1U<<4); //16MHZ

	/*Set I2C to standard mode, 100 khz clock*/
	I2C1->CCR = I2C_100khz;

	/*Set rise time*/
	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

	/*Enable I2C| module*/
	I2C1->CR1 |= CR1_PE;
}

void I2C1_byteRead(uint8_t dir_e, uint8_t dir_m, uint8_t *data){ //Cambiar por bool en lugar de void
	I2C1_burstRead(dir_e, dir_m, 1, data);
}

void I2C1_burstRead(uint8_t dir_e, uint8_t dir_m, uint16_t n, uint8_t *data){ //Cambiar por bool en lugar de void
	if(n<=0 || data == 0){
		return;
	}

	volatile int tmp;

	/*Initial reception on state*/
	I2C1->CR1 &= ~CR1_POS;
	I2C1->CR1 |= CR1_ACK;

	/*Wait until bus not busy*/
	while(I2C1->SR2 & (SR2_BUSY)){}

	/*Generate start*/
	I2C1->CR1 |= CR1_START;

	/*Wait until start flag is set*/
	while(!(I2C1->SR1 & (SR1_SB))){}

	/*Transmit slave address + write*/
	I2C1->DR = dir_e << 1;

	/*Wait until addr flag is set*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*Clear addr flag*/
	tmp = I2C1->SR2;

	/*Send memory addres*/
	I2C1->DR = dir_m;

	/*Wait until transmitter empty*/
	while(!(I2C1->SR1 & SR1_BTF)){}

	/*Generate restart*/
	I2C1->CR1 |= CR1_START;

	/*Wait until start flag is set*/
	while(!(I2C1->SR1 & SR1_SB)){}

	/*Transmit slave address + Read*/
	I2C1->DR = dir_e << 1 | 1;

	/*Wait until addr flag is set*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*cases*/

		if(n == 1){ //Un solo byte
			/*Disable Acknowledge*/
			I2C1->CR1 &= ~CR1_ACK;

			/*Clear addr flag*/
			tmp = I2C1->SR2;

			/*Generate stop*/
			I2C1->CR1 |= CR1_STOP;

			/*Wait until RXNE flag is set*/
			while(!(I2C1->SR1 & SR1_RXE)){}

			*data = I2C1->DR;
		}else if(n == 2){ //Dos bytes
			/*Disable Acknowledge*/
			I2C1->CR1 &= ~CR1_ACK;

			/*Enable POS*/
			I2C1->CR1 |= CR1_POS;

			/*Clear addr flag*/
			tmp = I2C1->SR2;

			/*Wait until transfer finished*/
			while(!(I2C1->SR1 & (SR1_BTF))){}

			/*Generate stop*/
			I2C1->CR1 |= CR1_STOP;

			*data++ = I2C1->DR;
			*data = I2C1->DR;
		}else{
			/*Clear addr flag*/
			tmp = I2C1->SR2;

			/*Receive the n-3 bytes*/
			while(n>3){
				/*Wait until RXNE flag is set*/
				while(!(I2C1->SR1 & SR1_RXE)){}
				*data++ = I2C1->DR;
				n--;
			}

			/*Wait until transfer finished*/
			while(!(I2C1->SR1 & (SR1_BTF))){}

			/*Disable Acknowledge*/
			I2C1->CR1 &= ~CR1_ACK;

			*data++ = I2C1->DR;

			/*Wait until transfer finished*/
			while(!(I2C1->SR1 & (SR1_BTF))){}

			/*Generate stop*/
			I2C1->CR1 |= CR1_STOP;

			*data++ = I2C1->DR;
			*data = I2C1->DR;
		}

		// Restore reception configuration
		I2C1->CR1 &= ~CR1_POS;
		I2C1->CR1 |= CR1_ACK;
}

void I2C1_burstWrite(uint8_t dir_e, uint8_t dir_m, uint16_t n, uint8_t *data){ //Cambiar por bool en lugar de void
	volatile int tmp;

	/*Wait until bus not busy*/
	while(I2C1->SR2 & (SR2_BUSY)){}

	/*Generate start*/
	I2C1->CR1 |= CR1_START;

	/*Wait until start flag is set*/
	while(!(I2C1->SR1 & (SR1_SB))){}

	/*Transmit slave address + write*/
	I2C1->DR = dir_e << 1;

	/*Wait until addr flag is set*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*Clear addr flag*/
	tmp = I2C1->SR2;

	/*Wait until transmitter empty*/
	while(!(I2C1->SR1 & SR1_TXE)){}

	/*Send memory addres*/
	I2C1->DR = dir_m;

	for(int i = 0; n>i; i++){
		/*Wait until transmitter empty*/
		while(!(I2C1->SR1 & SR1_TXE)){}

		/*Transmit data*/
		I2C1->DR = *data++;
	}

	/*Wait until transfer finished*/
	while(!(I2C1->SR1 & (SR1_BTF))){}

	/*Generate stop*/
	I2C1->CR1 |= CR1_STOP;
}
