#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_adc.h"

int read = 0;

int main() {

	GPIO_InitTypeDef		GPIO_InitStructure;
	ADC_InitTypeDef		ADC_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	ADC_StructInit(&ADC_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_1Cycles5);


	while (1) {

		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		read = ADC_GetConversionValue(ADC1);
		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		if (read > 2048) GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_SET);
		else GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_RESET);

	}
}