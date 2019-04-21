#include "ChargerFunctions.h"

void setDischarge(BMSconfigStructTypedef cfg, uint16_t allVoltage[12][12], bool allConnection[12][12], bool cellDischarge[12][8], uint8_t chargeRate) {

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