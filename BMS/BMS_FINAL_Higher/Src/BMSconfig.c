#include "BMSconfig.h"

void loadConfig(BMSconfigStructTypedef* cfg) {

	//change this
	cfg->numOfICs = 12;
	for(int i = 0; i<12; i++){
		cfg->address[i] = i;
	}
	// cfg->address[0] = 0;
	// cfg->address[1] = 1;

	cfg->numOfCellInputs = 12; 
	cfg->numOfCellsPerIC = 8;
	cfg->numOfTempPerIC = 4;

	cfg->OV_threshold = 42000;
	cfg->UV_threshold = 25000;

	cfg->LUV_threshold = 20000;
	cfg->HUV_threshold = 25000;
	//charge to 4.14V, lower charge current, discharge cells above 4.16V
	//exceed 4.18V, stop charging entirely, discharge to 4.15V
	//exceed 4.2V, fault prevents discharging

	//cell voltage limit 4.16V
	//full current or no current
	//exceed 4.17 on any cell, stop charging, discharge that one cell to 4.15
	//start charging again 

	// if there is a big enough delta above minimum, stop charging so that you can actually lower voltage 

	//UV FAULT 2.5, check ESF
	//if cells are below 3, don't balance

	cfg->slowCharge_threshold = 41400;
	cfg->stopCharge_threshold = 41800;
	cfg->max_difference = 2000;
	cfg->balancing_difference = 500;
	cfg->start_scaling = 41000;
	cfg->stop_scaling = 41600;
	cfg->scale_to = 100;

	cfg->invalidPECcount = 5;

	cfg->dischargeTime = 500; // ms

	cfg->normalCurrent = 0x003E; // 6.2A 
	cfg->lowerCurrent = 0x000A; // 1A
	cfg->chargerVoltage = 0xFA0; // 400V

	// 0: 422Hz, 1: 27kHz, 2: 7kHz, 3: 26Hz, 4: 1kHz, 5: 14kHz, 6: 3kHz, 7: 2kHz
	cfg->ADCConversionRate = 0;

	cfg->ADCModeOption = 1;
	
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
	