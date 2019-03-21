/*
	All variables below are required for the functions to run.
	
	Functions:
		int setChargeDischarge (int cellsDischarge[numBoards][cellsPerBoard], int *chargeCurrent, float cellVoltage[numBoards][cellsPerBoard])
			@brief - when function returns, the array cellsDischarge will be modified based on the new data as follows:
					-- Every cell: if the voltage is more than balancingDifference above the minimum cell voltage, set to discharge
					-- If any cell exceeds lowerVoltage_Threshold lower charge current to lowerCurrent and continue balancing
					-- If any cell exceeds higherVoltage_Threshold: Set charge current to 0 and continue balancing
					-- If any cell exceeds BMSFault_Threshold: Throw BMS Fault
			@return - 1 if there is a BMS fault, 0 if not
		
		float getLowestVoltage(float cellVoltage[numBoards][cellsPerBoard])
			@brief - finds and returns the lowest cell voltage
			@return - the lowest voltage of all cells
		
		void getChargerTxData()
			@brief - calculates and sets the CAN transmission data for the charger

	Author: Evan Chen
 */

/*********************************************************** Variables **********************************************************/

/************** Voltage Thresholds *******************/
float BMSFault_Threshold = 4.2;				// max voltage (V) before BMS Fault is thrown
float lowerVoltage_Threshold = 4.1;			// max voltage (V) before charge current is lowered
float higherVoltage_Threshold = 4.185;		// max voltage (V) before charge current is set to 0
float balancingDifference = 0.05;			// max difference in voltage (V) between current cell and cell with lowest voltage before current cell discharges
/************* End Voltage Thresholds ****************/


/*************** Charging Settings ********************/
/* hex value of ten times the current (A) in normal operation(when no cell is above lowerVoltage_Threshold) */
uint16_t normalCurrent = 0x000F;		// 1.5 A

/* hex value of ten times the current (A) when any cell exceeds lowerVoltage_Threshold */
uint16_t lowerCurrent = 0x000A;			// 1 A

/* hex value of ten times the voltage (V) of the charger */
uint16_t chargerVoltage = 0x0028;		// 4 V
/************** End Charging Settings *****************/


/************* Battery and Cell Numbers **************/
int numBoards = 12;
int cellsPerBoard = 8;
int cellsDischarge[12][8];			// 1 for discharging, 0 for not discharging
/************* End Battery and Cell Numbers **********/


/******************** Other **************************/
int chargeCurrent;		// flag used by program (2 for normal current, 1 for lower current, 0 for not charging)
uint8_t CANtx[8]		// CAN transmission data for the charger
/******************* End Other ***********************/

/*********************************************************** End Variables ******************************************************/

int setChargeDischarge (int cellsDischarge[numBoards][cellsPerBoard], int *chargeCurrent, float cellVoltage[numBoards][cellsPerBoard]);
float getLowestVoltage(float cellVoltage[numBoards][cellsPerBoard]);
float getLowestVoltage(float cellVoltage[numBoards][cellsPerBoard]);
void getChargerTxData();

/*
	@brief - when function returns, the array cellsDischarge will be modified based on the new data as follows:
				-- Every cell: if the voltage is more than balancingDifference above the minimum cell voltage, set to discharge
				-- If any cell exceeds lowerVoltage_Threshold lower charge current to lowerCurrent and continue balancing
				-- If any cell exceeds higherVoltage_Threshold: Set charge current to 0 and continue balancing
				-- If any cell exceeds BMSFault_Threshold: Throw BMS Fault
	@return - 1 if there is a BMS fault, 0 if not
*/
int setChargeDischarge (int cellsDischarge[numBoards][cellsPerBoard], int *chargeCurrent, float cellVoltage[numBoards][cellsPerBoard]) {
	float lowestVoltage = getLowestVoltage(cellVoltage);			// get lowest voltage of all cells
	chargeCurrent = 2;												// initialize the charging current to normal operation
	
	/* iterate through every cell */
	for (int board = 0; board < numBoards; board++) {
		for (int cell = 0; cell < cellsPerBoard; cell++) {
			/* get the voltage of current cell */
			float voltage = cellVoltage[board][cell];

			/* If any cell exceeds BMSFault_Threshold2 Throw BMS Fault */
			if (voltage > BMSFault_Threshold) {
				chargeCurrent = 0;
				return 1;
			}

			/* Set balancing: if the cell voltage is more than the set balancingDifference above the minimum cell voltage, set to discharge */
			if (voltage > (lowestVoltage + balancingDifference)) {
				cellsDischarge[board][cell] = 1;
			}

			/* If any cell exceeds higherVoltage_Threshold: Set charge current to 0 and continue balancing.
			 * If any cell exceeds lowerVoltage_Threshold: lower charge current to lowerCurrent and continue balancing
			 */
			if (voltage > higherVoltage_Threshold) {
				chargeCurrent = 0;
			} else if (voltage > lowerVoltage_Threshold && chargeCurrent != 0) {
				chargeCurrent = 1;
			}
		}
	 }

	 return 0;
}


/*
	@brief - finds and returns the lowest cell voltage
	@return - the lowest voltage of all cells
*/
float getLowestVoltage(float cellVoltage[numBoards][cellsPerBoard]) {
	/* contains the current lowest voltage foud */
	float low = cellVoltage[0][0];

	/* iterate through all cells and get the smallest voltage */
	for (int i = 0; i < numBoards; i++) {
		for (int j = 0; j < cellsPerBoard; j++) {
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
void getChargerTxData() {
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