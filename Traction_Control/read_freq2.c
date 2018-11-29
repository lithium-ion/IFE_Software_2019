/*	read_freq2.c
	Test program for reading the frequency of two square wave input signals
	Calculates % difference (slip ratio) between freq1 and freq2 

	This method of determining frequency should NOT be used in the final traction control program
	This program is slow and not as accurate as we want
	A better method is to configure the input pins as a timer channel running in input capture mode
	What is preventing us from using this method is being able to properly define an interrupt handler
*/

#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_tim.h"
#include "inc/misc.h"

int input1 = 0;
int lastinput1 = 0;
int delta1 = 0;
int freq1 = 0;
int ema1 = 0;
int lastema1 = 0;
int input2 = 0;
int lastinput2 = 0;
int delta2 = 0;
int freq2 = 0;
int ema2 = 0;
int lastema2 = 0;
int percent = 0;
int ema3 = 0;
int lastema3 = 0;

int main() {

	// Create necessary variables
	GPIO_InitTypeDef				GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	TIM_TimeBaseStructInit(&TIM_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Configure PA0 as output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure PA3 as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure PA2 as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure TIM2
	TIM_InitStructure.TIM_Prescaler = 0; // change this
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
	TIM_Cmd(TIM2, ENABLE);

	// Configure TIM3
	TIM_InitStructure.TIM_Prescaler = 0; // change this
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseInit(TIM3, &TIM_InitStructure);
	TIM_Cmd(TIM3, ENABLE);

	while (1) {	

		input1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
		input2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
	
		// check for rising edge of input1
		if (input1 == 1) {
			if (lastinput1 == 0) {
				delta1 = TIM_GetCounter(TIM2);
				TIM_SetCounter(TIM2, 0x0000);
			}
		}

		// check for rising edge of input2
		if (input2 == 1) {
			if (lastinput2 == 0) {
				delta2 = TIM_GetCounter(TIM3);
				TIM_SetCounter(TIM3, 0x0000);
			}
		}		

		freq1 = 8000000 / delta1;
		freq2 = 8000000 / delta2;

		// use an exponential moving average for freq1 and freq2
		if (lastema1 != 0) ema1 =  (freq1/4) + (3*lastema1/4);
		else ema1 = freq1;

		if (lastema2 != 0) ema2 =  (freq2/4) + (3*lastema2/4);
		else ema2 = freq2;

		percent = 100 * (ema1 - ema2) / ema2; // slip ratio

		if (percent < 0) { percent = -percent; }

		// use an exponential moving average for percent
		if (lastema3 != 0) ema3  =  (percent/4) + (3*lastema3/4);
		else ema3 = percent;

		if (ema3 > 5) {
			GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
		}
		else {
			GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
		}

		lastinput1 = input1;
		lastema1 = ema1;
		lastinput2 = input2;
		lastema2 = ema2;
		lastema3 = ema3;
		
	}
}