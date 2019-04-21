#include "ChargerFunctions.h"

void setDischarge(BMSconfigStructTypedef cfg, uint16_t allVoltage[12][12], bool allConnection[12][12], bool cellDischarge[12][8], uint8_t *chargeRate) {

	uint16_t lowestVoltage = getLowestVoltage(cfg, allVoltage); // gets lowest voltage of all cells
	chargeRate = 2;												// initialize the charging current to normal operation
	
	/* iterate through every cell */
	for (int board = 0; board < cfg.numOfICs; board++) {
		for (int cell = 0; cell < cfg.numOfCellsPerIC; cell++) {
			/* get the voltage of current cell */
			uint16_t voltage = allVoltage[board][cell];

			/* If any cell exceeds BMSFault_Threshold2 Throw BMS Fault */
			if (voltage > cfg.OV_threshold) {
				chargeRate = 0;
			}

			/* Set balancing: if the cell voltage is more than the set balancingDifference above the minimum cell voltage, set to discharge */
			if (voltage > (lowestVoltage + cfg.balancing_difference)) {
				cellDischarge[board][cell] = 1;
			}

			/* If any cell exceeds higherVoltage_Threshold: Set charge current to 0 and continue balancing.
			 * If any cell exceeds lowerVoltage_Threshold: lower charge current to lowerCurrent and continue balancing
			 */
			if (voltage > cfg.stopCharge_threshold)
				chargeRate = 0;
			else if (voltage > cfg.slowCharge_threshold && chargeRate != 0) 
				chargeRate = 1;
		}
	 }
}


uint16_t getLowestVoltage(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][8]) {
	/* contains the current lowest voltage found */
	uint16_t low = cellVoltage[0][0];

	/* iterate through all cells and get the smallest voltage */
	for (int i = 0; i < cfg.numOfICs; i++) {
		for (int j = 0; j < cfg.numOfCellsPerIC; j++) {
			if (low > cellVoltage[i][j]) {
				low = cellVoltage[i][j];
			}
		}
	}
	return low;
}

/*
	@brief - calculates and sets the CAN transmission data for the charger
*/
void setChargerTxData(uint8_t CANtx[8], uint8_t chargeCurrent, uint16_t chargerVoltage, uint16_t lowerCurrent, uint16_t normalCurrent) {
	/* voltage data (hex value of desired voltage (V) times 10)*/
	CANtx[0] = (uint8_t)(chargerVoltage >> 8);
	CANtx[1] = (uint8_t)chargerVoltage;

	/* set the current data (hex value of desired current (A) times 10) */
	switch (chargeCurrent) {
		case 1:
			/* lower current */
			CANtx[2] = (uint8_t)(lowerCurrent >> 8);
			CANtx[3] = (uint8_t)lowerCurrent;
			break;

		case 2:
			/* normal current */
			CANtx[2] = (uint8_t)(normalCurrent >> 8);
			CANtx[3] = (uint8_t)normalCurrent;
			break;

		default:
			/* no current */
			CANtx[2] = 0x00;
			CANtx[3] = 0x00;
	}

	/* these data bytes are not used */
	CANtx[4] = 0x00;
	CANtx[5] = 0x00;
	CANtx[6] = 0x00;
	CANtx[7] = 0x00;

	return;
}