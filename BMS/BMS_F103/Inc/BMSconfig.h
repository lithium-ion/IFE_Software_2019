#include "stm32f1xx.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

typedef struct {
	bool GPIO5PulldownOff;
	bool GPIO4PulldownOff;
	bool GPIO3PulldownOff;
	bool GPIO2PulldownOff;
	bool GPIO1PulldownOff;
	bool ReferenceOn;
	bool ADCModeOption;
	uint16_t UndervoltageComparisonVoltage;
	uint16_t OvervoltageComparisonVoltage;
	bool DischargeCell1;
	bool DischargeCell2;
	bool DischargeCell3;
	bool DischargeCell4;
	bool DischargeCell5;
	bool DischargeCell6;
	bool DischargeCell7;
	bool DischargeCell8;
	bool DischargeCell9;
	bool DischargeCell10;
	bool DischargeCell11;
	bool DischargeCell12;
	uint8_t DischargeTimeoutValue;
	
	uint8_t ADCMode;
	bool DischargePermitted;
} BMSconfigStructTypedef;

void loadConfig(BMSconfigStructTypedef* config);