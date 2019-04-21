#include "BMSconfig.h"

void loadConfig(BMSconfigStructTypedef* cfg) {
		
	cfg->numOfICs = 1;
	cfg->address[0] = 0;

	cfg->numOfCellInputs = 12; // this should never change
	cfg->numOfCellsPerIC = 8;
	cfg->numOfTempPerIC = 4;

	cfg->OV_threshold = 42000;
	cfg->slowCharge_threshold = 41000;
	cfg->stopCharge_threshold = 41850;
	
	// 0: 422Hz, 1: 27kHz, 2: 7kHz, 3: 26Hz, 4: 1kHz, 5: 14kHz, 6: 3kHz, 7: 2kHz
	cfg->ADCConversionRate = 0;

	cfg->ADCModeOption = 0;
	
	// chip and code should share OV and UV thresholds
	
	//cfg->ADCModeOption = (config.ADCConversionRate) & 0b011;
	//cfg->ADCMode = (config.ADCConversionRate) & 0b100;
	
	cfg->GPIO5PulldownOff = 1;
	cfg->GPIO4PulldownOff = 1;
	cfg->GPIO3PulldownOff = 1;
	cfg->GPIO2PulldownOff = 1;
	cfg->GPIO1PulldownOff = 1;
	
	cfg->ReferenceOn = 1; // minimizes time between conversions
	
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
	