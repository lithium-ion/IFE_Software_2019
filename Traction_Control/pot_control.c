/*	pot_control.c
	Test program for sending SPI values to MCP4131 digital potentiometer. 
*/

#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_spi.h"
#include "inc/misc.h"

void digitalPotWrite(int value) {

	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // slave select pin pulled low
	SPI_I2S_SendData(SPI1, value);												
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // wait until transmit buffer is empty
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET); // wait until transmit is complete
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}


int main() {

	// Create necessary variables
	GPIO_InitTypeDef		GPIO_InitStructure;
	SPI_InitTypeDef		SPI_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	SPI_StructInit(&SPI_InitStructure);

	// Enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// Set PA4 as output (SS)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Set PA5 as AF output (SCK)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // needs to be 50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Set PA7 as AF Output (MOSI)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // needs to be 50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure SPI
	SPI_InitStructure.SPI_Direction = SPI_Direction_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b; // has to be 16-bit if using digitalPotWrite() function
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // 8MHz/32=250kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	//Enable SPI
	SPI_Cmd(SPI1, ENABLE);

	while (1) {
		digitalPotWrite(85); // can be any value from 0-128
		for (int i = 0; i < 500; i++); // delay
	}

}