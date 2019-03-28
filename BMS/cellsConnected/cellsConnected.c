/* 
	IMPORTATNT: Put these into the command code enum in LTC6811.h
		ADOWPullup = 0x269
		ADOWPulldown = 0x229
*/

/*
	@return true if all cells are connected on every board, false if any cell is disconnected.
	@output sets cellConnected[i] to 1 if cell i is connected. cellConnected[i] is set to 0 if cell i is disconnected.
*/
bool checkAllConnected(bool cellConnected[12][12]) {
	return checkBoardConnected(1, cellConnected, 0) && checkBoardConnected(2, cellConnected, 1) &&
			checkBoardConnected(3, cellConnected, 2) && checkBoardConnected(4, cellConnected, 3) &&
			checkBoardConnected(5, cellConnected, 4) && checkBoardConnected(6, cellConnected, 5) &&
			checkBoardConnected(7, cellConnected, 6) && checkBoardConnected(8, cellConnected, 7) &&
			checkBoardConnected(9, cellConnected, 8) && checkBoardConnected(10, cellConnected, 9) &&
			checkBoardConnected(11, cellConnected, 10) && checkBoardConnected(12, cellConnected, 11);
}

/*
	@return true if all cells are connected on a certain board, false if any cell is disconnected.
	@output sets cellConnected[board][i] to 1 if cell i is connected. cellConnected[board][i] is set to 0 if cell i is disconnected.
*/
bool checkBoardConnected(uint8_t address, bool cellConnected[12][12], int boardIndex) {
	uint16_t pullupVoltages[12];
	uint16_t pulldownVoltages[12];
	bool returnVal = true;

	/* get the pullup voltages */
	sendAddressCommand(ADOWPullup, address);
	HAL_Delay(10);
	sendAddressCommand(ADOWPullup, address);
	HAL_Delay(10);
	readCellVoltage(address, pullupVoltages);
	HAL_Delay(10);

	/* get the pulldown voltages */
	sendAddressCommand(ADOWPulldown, address);
	HAL_Delay(10);
	sendAddressCommand(ADOWPulldown, address);
	HAL_Delay(10);
	readCellVoltage(address, pulldownVoltages);

	/* calculate the difference between the two voltages for every cell except for cell 1 (index 0) and cell 12 (index 11)*/
	for (int i = 1; i <= 10; i++) {
		if(((float)(pullupVoltages[i+1] - pulldownVoltages[i+1]) / 10000) < -0.4) {
			cellConnected[boardIndex][i] = 1;
		} else {
			cellConnected[boardIndex][i] = 0;
			returnVal = false;
		}
	}

	/* for cell 1, the relevant voltage to check is just pullup*/
	if (pullupVoltages[0] == 0) {
		cellConnected[boardIndex][0] = 0;
	} else {
		cellConnected[boardIndex][0] = 1;
	}

	/* for cell 12, the relevant voltage to check is just pulldown */
	if (pulldownVoltages[11] == 0) {
		cellConnected[boardIndex][11] = 0;
	} else {
		cellConnected[[boardIndex][11] = 1;
	}

	return returnVal;
}