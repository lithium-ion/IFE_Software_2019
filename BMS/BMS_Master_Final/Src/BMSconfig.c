#include "BMSconfig.h"

void loadConfig(BMSconfigStructTypedef* cfg) {
	
	// 422 Hz
	cfg->ADCModeOption = 0;
	cfg->ADCMode = 0;
	
	cfg->GPIO5PulldownOff = 1;
	cfg->GPIO4PulldownOff = 1;
	cfg->GPIO3PulldownOff = 1;
	cfg->GPIO2PulldownOff = 1;
	cfg->GPIO1PulldownOff = 1;
	
	cfg->ReferenceOn = 1; // makes conversion faster
	
	cfg->UndervoltageComparisonVoltage = 0x000;
	cfg->OvervoltageComparisonVoltage = 0x000;
	
	cfg->DischargeCell1 = 0;
	cfg->DischargeCell2 = 0;
	cfg->DischargeCell3 = 0;
	cfg->DischargeCell4 = 0;
	cfg->DischargeCell5 = 0;
	cfg->DischargeCell6 = 0;
	cfg->DischargeCell7 = 0;
	cfg->DischargeCell8 = 0;
	cfg->DischargeCell9 = 0;
	cfg->DischargeCell10 = 0;
	cfg->DischargeCell11 = 0;
	cfg->DischargeCell12 = 0;
	
	cfg->DischargeTimeoutValue = 0x0;
}
	