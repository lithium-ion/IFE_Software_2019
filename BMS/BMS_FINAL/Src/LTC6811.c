/*
	LTC6811 Command Functions
	Matt Vlasaty
	March 29th, 2019
	
	General Functions
		sendBroadcastCommand: void sendBroadcastCommand(CommandCodeTypedef command);
			- sends specified write-only command to every LTC in the chain (ex: ADCV)
			- CommandCodeTypedef enum contains every command code used
		sendAddressCommand: void sendAddressCommand(CommandCodeTypedef command, uint8_t address);
			- sends specified write-only command to LTC with specified address
		readRegister: bool readRegister(CommandCodeTypedef command, uint8_t address, uint16_t *data);
			- reads register specified in command from specified board
			- ex: ReadCellVoltageRegisterGroup1to3, ReadAuxiliaryGroupA, ReadConfigurationRegisterGroup
			- returns true if received PEC matches calculated PEC
		
	Configuration Functions
		writeConfigAll: void writeConfigAll(BMSconfigStructTypedef cfg);
			- writes configuration data from BMSconfig struct into LTC configuration register
			- BMSconfig struct includes ADCMode, UV, OV, and discharge enable for each cell
			- BMSconfig struct also contains number of ICs and array of addresses used
		writeConfigAddress: void writeConfigAddress(BMSconfigStructTypedef cfg, uint8_t address);
			- this is only used for cellDischarge command
			- writes configuration data to LTC with specified address
		readConfig: bool readConfig(uint8_t address, uint8_t cfg[6]);
			- uses general readRegister function to check current state of LTC configuration register
			- this is mostly for testing purposes
			- returns true if received PEC matches calculated PEC (same as readRegister)
		
	Cell Voltage Functions
		readCellVoltage: bool readCellVoltage(uint8_t address, uint16_t cellVoltage[12]);
			- sends ADCV command that begins conversion for every cell to specified LTC
			- reads all cell voltage registers using general readRegister function
			- used cell inputs (1, 2, 3, 4, 7, 8, 9, 10) must be chosen from cellVoltage[12] later
			- returns true if received PEC matches calculated PEC for every register read
		readAllCellVoltages: bool readAllCellVoltages(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]);
			- stores cell number, cell voltage, and data valid bit into bmsData
			- returns true if no PEC for any register read for any board
	Cell Temperature Functions
		readCellTemp: bool readCellTemp(uint8_t address, uint16_t cellTemp[4], bool dcFault[4], bool tempFault[4]);
			- initiates ADC conversion for GPIO inputs connected to temperature sensors
			- reads auxiliary register groups using general readRegister function
			- converts measured voltage into temperature based on temperature sensor response
			- checks for disconnected temperature sensor and overtemperature faults for each LTC and stores results
		readAllCellTemps: bool readAllCellTemps(uint8_t numBoards, uint16_t cellTemp[numBoards][4], bool dcFault[numBoards][4], bool tempFault[numBoards][4]);
			- not tested
			- using order of ICs in BMSconfig, stores cell temperature for every cell on every board into 2D array
			- returns false if any readCellTemp returns false
	
	Cell Connection Functions
		checkCellConnection: void checkCellConnection(uint16_t cellVoltage[12], bool cellConnection[12]);
			- needs previously obtained cell voltage measurements
			- initiates ADOW command with PUP = 0 to source 100uA while measuring cell voltage
			- reads cell voltages with readCellVoltage function
			- compares previously measured values to open wire check values
			- if there is a significant drop in voltage (should be configurable), cell is disconnected
			- stores 1 into cellConnection if cell is connected
			- stores 0 into cellConnection if cell is disconnected
		checkAllCellConnections: 
	
	PEC Functions
		initPECTable: void initPECTable(void);
			- taken from LTC6811 datasheet
			- generates PEC look-up table
			- should be called on start-up
		calculatePEC: uint16_t calculatePEC(uint8_t len, uint8_t *data);
			- taken from LTC6811 datasheet
			- used when sending command to calculate necessary PEC bytes to follow command bytes
			- used when receiving data to compare received PEC with the PEC that should have been received based on data
			- returns uint16_t PEC value
			
	TODO:
		- checkCellConnection return value needs to disregard unused cell inputs
		- resolve issue with writeConfig not changing every bit in the first register group
		- test functions used to read from every board (readAllCellVoltages should be [12][8])
		- minor changes (using more user-defined constants, changing return values)
		
	NOTES:
		- writeConfig is called every loop for every LTC because dischargeCells is called every loop for every LTC
		- always take measurements, only send when not charging? (CAN messages for charging?)
		- every 'readAll' function starts with address = 0 and increases sequentially
		
		- in temp read, faults are determined but not transferred
		- temperature conversion works, but error is +/- 1 degree
*/


#include "LTC6811.h"

uint16_t pec15Table[256];
uint16_t CRC15_POLY = 0x4599;

void initPECTable(void) {
	
	uint16_t remainder;
	
	for (int i = 0; i < 256; i++) {
		remainder = i << 7;
		for (int bit = 8; bit > 0; bit--) {
			if (remainder & 0x4000) {
				remainder = ((remainder << 1));
				remainder = (remainder ^ CRC15_POLY);
			}
			else {
				remainder = ((remainder << 1));
			}
		}
		pec15Table[i] = remainder & 0xFFFF;
	}
};

void writeConfigAddress(BMSconfigStructTypedef cfg, uint8_t address) {

	uint8_t config[6];
	uint8_t *cmd;
	uint8_t cmd_len = 12;
	uint16_t PEC_return;
	uint8_t dummy[8];

	// readConfig(address, dummy);
	
	cmd = (uint8_t *)malloc(cmd_len*sizeof(uint8_t));
	
	config[0] = (uint8_t) (cfg.GPIO5PulldownOff << 7) | (cfg.GPIO4PulldownOff << 6) | (cfg.GPIO3PulldownOff << 5) | (cfg.GPIO2PulldownOff << 4) | (cfg.GPIO1PulldownOff << 3) | (cfg.ReferenceOn << 2) | (cfg.ADCModeOption);
	config[1] = (uint8_t) (cfg.UndervoltageComparisonVoltage & 0xFF);
	config[2] = (uint8_t) ((cfg.OvervoltageComparisonVoltage << 4) & 0xF0) | ((cfg.UndervoltageComparisonVoltage >> 8) & 0x0F);
	config[3] = (uint8_t) ((cfg.OvervoltageComparisonVoltage >> 4) & 0xFF);
	config[4] = (uint8_t) (cfg.DischargeCell8 << 7) | (cfg.DischargeCell7 << 6) | (cfg.DischargeCell6 << 5) | (cfg.DischargeCell5 << 4) | (cfg.DischargeCell4 << 3) | (cfg.DischargeCell3 << 2) | (cfg.DischargeCell2 << 1) | (cfg.DischargeCell1);
	config[5] = (uint8_t) ((cfg.DischargeTimeoutValue << 4) & 0xF0) | (cfg.DischargeCell12 << 3) | (cfg.DischargeCell11 << 2) | (cfg.DischargeCell10 << 1) | (cfg.DischargeCell9);
	
	cmd[0] = (uint8_t) (0x80 | ((address << 3) & 0x78) | ((WriteConfigurationRegisterGroup >> 8) & 0x07));
	cmd[1] = (uint8_t) (WriteConfigurationRegisterGroup & 0xFF);
	
	PEC_return = calculatePEC(2, cmd);
	
	cmd[2] = (PEC_return >> 8) & 0xFF;
	cmd[3] = PEC_return & 0xFF;

	cmd[4] = config[0];
	cmd[5] = config[1];
	cmd[6] = config[2];
	cmd[7] = config[3];
	cmd[8] = config[4];
	cmd[9] = config[5];
	
	PEC_return = calculatePEC(6, cmd + 4);

	cmd[10] = (PEC_return >> 8) & 0xFF;
	cmd[11] = PEC_return & 0xFF;
	
	SPIWrite(cmd, cmd_len);

	readConfig(address, dummy);
	
	free(cmd);

};

void writeConfigAll(BMSconfigStructTypedef cfg) {

	uint32_t delay = 15;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	while(delay--);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	for (uint8_t i = 0; i < cfg.numOfICs; i++) {
		writeConfigAddress(cfg, cfg.address[i]);
	}
};

bool readCellVoltage(uint8_t address, uint16_t cellVoltage[12]) {
	
	bool PEC_check = false;
	bool dataValid = true;
	uint16_t *voltage;
	
	voltage = (uint16_t *)malloc(12*sizeof(uint16_t));
	
	// sendAddressCommand(0x711, address); // clear cell voltage groups
	// sendAddressCommand(StartCellVoltageADCConversionAll, address); // start conversion for every cell
	// sendAddressCommand(0x261, address); // 1 and 7
	// sendAddressCommand(0x264, address); // 4 and 10

	// HAL_Delay(50); // conversion time is 12.8ms at 422Hz, so wait 15ms
	
	PEC_check = readRegister(ReadCellVoltageRegisterGroup1to3, address, voltage);
	//dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadCellVoltageRegisterGroup4to6, address, voltage);
	dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadCellVoltageRegisterGroup7to9, address, voltage);
	dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadCellVoltageRegisterGroup10to12, address, voltage);
	dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadCellVoltageRegisterGroup1to3, address, voltage);
	dataValid = dataValid & PEC_check;
	
	for (uint8_t i = 0; i < 12; i++) {
		cellVoltage[i] = voltage[i];
	}
	
	free(voltage);
	
	return(dataValid);
	
};

bool readAllCellVoltages(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]) {

	/*
	uint16_t boardVoltage[cfg.numOfCellInputs];
	bool PEC_check[cfg.numOfICs];
	bool dataValid = true;

	for (uint8_t board = 0; board < cfg.numOfICs; board++) {

		//read voltage of every cell input (1-12) for a specific address, store in boardVoltage
		PEC_check[board] = readCellVoltage(cfg.address[board], boardVoltage);

		//store cell number and valid data bit in bmsData
		for (uint8_t cell = 0; cell < cfg.numOfCellsPerIC; cell++) {
			bmsData[(board * cfg.numOfCellsPerIC) + cell][0] = (uint8_t) ((board * cfg.numOfCellsPerIC) + cell + 1); //cell number
			bmsData[(board * cfg.numOfCellsPerIC) + cell][1] = (uint8_t) (0x02 & (PEC_check[board] << 1)); //valid data bit in status byte
		}

		//store cell voltage in bmsData
		bmsData[(board * cfg.numOfCellsPerIC) + 0][2] = (uint8_t) ((boardVoltage[0] >> 8) & 0xFF); //cell 1, voltage H
		bmsData[(board * cfg.numOfCellsPerIC) + 0][3] = (uint8_t) (boardVoltage[0] & 0xFF); //cell 1, voltage L
		bmsData[(board * cfg.numOfCellsPerIC) + 1][2] = (uint8_t) ((boardVoltage[1] >> 8) & 0xFF); //cell 2, voltage H
		bmsData[(board * cfg.numOfCellsPerIC) + 1][3] = (uint8_t) (boardVoltage[1] & 0xFF); //cell 2, voltage L
		bmsData[(board * cfg.numOfCellsPerIC) + 2][2] = (uint8_t) ((boardVoltage[2] >> 8) & 0xFF); //cell 3, voltage H
		bmsData[(board * cfg.numOfCellsPerIC) + 2][3] = (uint8_t) (boardVoltage[2] & 0xFF); //cell 3, voltage L
		bmsData[(board * cfg.numOfCellsPerIC) + 3][2] = (uint8_t) ((boardVoltage[3] >> 8) & 0xFF); //cell 4, voltage H
		bmsData[(board * cfg.numOfCellsPerIC) + 3][3] = (uint8_t) (boardVoltage[3] & 0xFF); //cell 4, voltage L 
		bmsData[(board * cfg.numOfCellsPerIC) + 4][2] = (uint8_t) ((boardVoltage[6] >> 8) & 0xFF); //cell 7, voltage H
		bmsData[(board * cfg.numOfCellsPerIC) + 4][3] = (uint8_t) (boardVoltage[6] & 0xFF); //cell 7, voltage L 
		bmsData[(board * cfg.numOfCellsPerIC) + 5][2] = (uint8_t) ((boardVoltage[7] >> 8) & 0xFF); //cell 8, voltage H
		bmsData[(board * cfg.numOfCellsPerIC) + 5][3] = (uint8_t) (boardVoltage[7] & 0xFF); //cell 8, voltage L
		bmsData[(board * cfg.numOfCellsPerIC) + 6][2] = (uint8_t) ((boardVoltage[8] >> 8) & 0xFF); //cell 9, voltage H
		bmsData[(board * cfg.numOfCellsPerIC) + 6][3] = (uint8_t) (boardVoltage[8] & 0xFF); //cell 9, voltage L
		bmsData[(board * cfg.numOfCellsPerIC) + 7][2] = (uint8_t) ((boardVoltage[9] >> 8) & 0xFF); //cell 10, voltage H
		bmsData[(board * cfg.numOfCellsPerIC) + 7][3] = (uint8_t) (boardVoltage[9] & 0xFF); //cell 10, voltage L
	
	}

	for (uint8_t board = 0; board < cfg.numOfICs; board++) {
		if (PEC_check[board] == 0)
			dataValid = false;
	}

	return dataValid; //return true if no PEC errors for any board
	*/

	uint16_t boardVoltage[12];
	bool PEC_check[12];
	bool dataValid = true;

	sendBroadcastCommand(ClearRegisters);
	sendBroadcastCommand(StartCellVoltageADCConversionAll);
	HAL_Delay(20);

	uint32_t delay = 15;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	while(delay--);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	for (uint8_t board = 0; board < 12; board++) {

		//read voltage of every cell input (1-12) for a specific address, store in boardVoltage
		PEC_check[board] = readCellVoltage(board, boardVoltage);

		//store cell number and valid data bit in bmsData
		for (uint8_t cell = 0; cell < 8; cell++) {
			bmsData[(board * 8) + cell][0] = (uint8_t) ((board * 8) + cell + 1); //cell number
			bmsData[(board * 8) + cell][1] = (uint8_t) (0x02 & (PEC_check[board] << 1)); //valid data bit in status byte
		}

		//store cell voltage in bmsData
		bmsData[(board * 8) + 0][2] = (uint8_t) ((boardVoltage[0] >> 8) & 0xFF); //cell 1, voltage H
		bmsData[(board * 8) + 0][3] = (uint8_t) (boardVoltage[0] & 0xFF); //cell 1, voltage L
		bmsData[(board * 8) + 1][2] = (uint8_t) ((boardVoltage[1] >> 8) & 0xFF); //cell 2, voltage H
		bmsData[(board * 8) + 1][3] = (uint8_t) (boardVoltage[1] & 0xFF); //cell 2, voltage L
		bmsData[(board * 8) + 2][2] = (uint8_t) ((boardVoltage[2] >> 8) & 0xFF); //cell 3, voltage H
		bmsData[(board * 8) + 2][3] = (uint8_t) (boardVoltage[2] & 0xFF); //cell 3, voltage L
		bmsData[(board * 8) + 3][2] = (uint8_t) ((boardVoltage[3] >> 8) & 0xFF); //cell 4, voltage H
		bmsData[(board * 8) + 3][3] = (uint8_t) (boardVoltage[3] & 0xFF); //cell 4, voltage L 
		bmsData[(board * 8) + 4][2] = (uint8_t) ((boardVoltage[6] >> 8) & 0xFF); //cell 7, voltage H
		bmsData[(board * 8) + 4][3] = (uint8_t) (boardVoltage[6] & 0xFF); //cell 7, voltage L 
		bmsData[(board * 8) + 5][2] = (uint8_t) ((boardVoltage[7] >> 8) & 0xFF); //cell 8, voltage H
		bmsData[(board * 8) + 5][3] = (uint8_t) (boardVoltage[7] & 0xFF); //cell 8, voltage L
		bmsData[(board * 8) + 6][2] = (uint8_t) ((boardVoltage[8] >> 8) & 0xFF); //cell 9, voltage H
		bmsData[(board * 8) + 6][3] = (uint8_t) (boardVoltage[8] & 0xFF); //cell 9, voltage L
		bmsData[(board * 8) + 7][2] = (uint8_t) ((boardVoltage[9] >> 8) & 0xFF); //cell 10, voltage H
		bmsData[(board * 8) + 7][3] = (uint8_t) (boardVoltage[9] & 0xFF); //cell 10, voltage L
	
	}

	for (uint8_t board = 0; board < 12; board++) {
		if (PEC_check[board] == 0)
			dataValid = false;
	}

	return dataValid; //return true if no PEC errors for any board
}

bool readCellTemp(uint8_t address, uint16_t cellTemp[4], bool dcFault[4], bool tempFault[4]) {
	
	bool PEC_check = false;
	bool dataValid = true;
	uint16_t *temp;
	double dummy[4];
	double conversion[4];
	
	temp = (uint16_t *)malloc(4*sizeof(uint16_t));
	
	sendAddressCommand(StartCellTempVoltageADCConversionAll, address);
	
	HAL_Delay(50); // conversion time is 12.8ms at 422Hz, so wait 15ms
	
	PEC_check = readRegister(ReadAuxiliaryGroupA, address, temp);
	//dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadAuxiliaryGroupB, address, temp);
	dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadAuxiliaryGroupA, address, temp);
	dataValid = dataValid & PEC_check;
	
	for (uint8_t i = 0; i < 4; i++) {
		if ((temp[i] > 24400) || (temp[i] < 13000)) {
			dcFault[i] = true;
			cellTemp[i] = 0x0000;
		}
		else
			dcFault[i] = false;
		
		if ((temp[i] >= 13000) && (temp[i] < 15100)) {
			tempFault[i] = true;
			cellTemp[i] = 0xFFFF;
		}
		else
			tempFault[i] = false;
		
		if ((temp[i] >= 15100) && (temp[i] <= 24400)) {
			dummy[i] = (double) temp[i] / 10000; // convert from ADC value to voltage
			// convert from voltage to temperature
			conversion[i] = 37735 - 113923 * dummy[i] + 142663 * pow(dummy[i], 2) - 94465 * pow(dummy[i], 3) + 34799 * pow(dummy[i], 4) - 6749 * pow(dummy[i], 5) + 537.11 * pow(dummy[i], 6);
			conversion[i] = 500 * (conversion[i] + 20); // map value to larger range
			cellTemp[i] = (uint16_t) conversion[i];
		}
	}
	
	free(temp);
	
	return (dataValid);
	
};

bool readAllCellTemps(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]) {

	uint16_t boardTemp[4];
	bool boardDCFault[4];
	bool boardTempFault[4];
	bool PEC_check[12];
	bool dataValid = true;

	for (uint8_t board = 0; board < 12; board++) {

		//read temperature, check for OT and temp DC
		PEC_check[board] = readCellTemp(board, boardTemp, boardDCFault, boardTempFault);

		//store OT and temp DC bits in status byte
		for (uint8_t cell = 0; cell < 8; cell++) {
			bmsData[(board * 8) + cell][1] &= (PEC_check[board] << 1);
			bmsData[(board * 8) + cell][1] |= (boardTempFault[cell / 2] << 4);
			bmsData[(board * 8) + cell][1] |= (boardDCFault[cell / 2] << 3);
		}

		//store cell temperature in bmsData
		bmsData[(board * 8) + 0][4] = (uint8_t) ((boardTemp[0] >> 8) & 0xFF); //temp1 H
		bmsData[(board * 8) + 0][5] = (uint8_t) (boardTemp[0] & 0xFF); //temp1 L
		bmsData[(board * 8) + 1][4] = (uint8_t) ((boardTemp[0] >> 8) & 0xFF); //temp1 H
		bmsData[(board * 8) + 1][5] = (uint8_t) (boardTemp[0] & 0xFF); //temp1 L
		bmsData[(board * 8) + 2][4] = (uint8_t) ((boardTemp[1] >> 8) & 0xFF); //temp2 H
		bmsData[(board * 8) + 2][5] = (uint8_t) (boardTemp[1] & 0xFF); //temp2 L
		bmsData[(board * 8) + 3][4] = (uint8_t) ((boardTemp[1] >> 8) & 0xFF); //temp2 H
		bmsData[(board * 8) + 3][5] = (uint8_t) (boardTemp[1] & 0xFF); //temp2 L
		bmsData[(board * 8) + 4][4] = (uint8_t) ((boardTemp[2] >> 8) & 0xFF); //temp3 H
		bmsData[(board * 8) + 4][5] = (uint8_t) (boardTemp[2] & 0xFF); //temp3 L
		bmsData[(board * 8) + 5][4] = (uint8_t) ((boardTemp[2] >> 8) & 0xFF); //temp3 H
		bmsData[(board * 8) + 5][5] = (uint8_t) (boardTemp[2] & 0xFF); //temp3 L
		bmsData[(board * 8) + 6][4] = (uint8_t) ((boardTemp[3] >> 8) & 0xFF); //temp4 H
		bmsData[(board * 8) + 6][5] = (uint8_t) (boardTemp[3] & 0xFF); //temp4 L
		bmsData[(board * 8) + 7][4] = (uint8_t) ((boardTemp[3] >> 8) & 0xFF); //temp4 H
		bmsData[(board * 8) + 7][5] = (uint8_t) (boardTemp[3] & 0xFF); //temp4 L
	}
	
	for (uint8_t board = 0; board < 12; board++) {
		if (PEC_check[board] == 0)
			dataValid = false;
	}

	return dataValid;
}

bool readConfig(uint8_t address, uint8_t cfg[8]) {
	
	uint16_t *config;
	bool dataValid = false;
	
	config = (uint16_t *)malloc(4*sizeof(uint16_t));
	
	dataValid = readRegister(ReadConfigurationRegisterGroup, address, config);
	
	cfg[0] = (uint8_t) ((config[0] >> 8) & 0xFF);
	cfg[1] = (uint8_t) (config[0] & 0xFF);
	cfg[2] = (uint8_t) ((config[1] >> 8) & 0xFF);
	cfg[3] = (uint8_t) (config[1] & 0xFF);
	cfg[4] = (uint8_t) ((config[2] >> 8) & 0xFF);
	cfg[5] = (uint8_t) (config[2] & 0xFF);
	cfg[6] = (uint8_t) ((config[3] >> 8) & 0xFF);
	cfg[7] = (uint8_t) (config[3] & 0xFF);
	
	return dataValid;
}

/*bool checkCellConnection(uint8_t address, uint16_t cellVoltage[12], bool cellConnection[12]) {
		

	
	for (uint8_t i = 0; i < 12; i++) {
		// if voltage fell by > 100mV
		if ((cellVoltage[i] - ADOWvoltage[i]) > 1000) {
			cellConnection[i] = 0; // cell is disconnected
			disconnect = true;
		}
		else {
			cellConnection[i] = 1;
		}
	}
	return disconnect;
}*/

bool checkAllCellConnections(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]) {
	
	uint16_t ADOWvoltage[cfg.numOfCellInputs];
	uint16_t cellVoltage;
	bool disconnect = false;

	//at least 2
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);

	HAL_Delay(15);

	for (uint8_t board = 0; board < cfg.numOfICs; board++) {

		readCellVoltage(cfg.address[board], ADOWvoltage);

		for (uint8_t cell = 0; cell < cfg.numOfCellsPerIC; cell++) {

			cellVoltage = 0;
			cellVoltage = (uint16_t) (bmsData[(board * cfg.numOfCellsPerIC) + cell][2]);
			cellVoltage = cellVoltage << 8;
			cellVoltage += (uint16_t) (bmsData[(board * cfg.numOfCellsPerIC) + cell][3]);

			if ((cellVoltage - ADOWvoltage[cell]) > 1000)
				bmsData[(board * cfg.numOfCellsPerIC) + cell][1] &= 0xFE;
			else
				bmsData[(board * cfg.numOfCellsPerIC) + cell][1] |= 0x01;
		}
	}

	return 0;
}

bool dischargeCellGroups(BMSconfigStructTypedef config, bool cellDischarge[12][8]) {
	
	BMSconfigStructTypedef *cfg;

	cfg = &config;

	for (uint8_t i = 0; i < config.numOfICs; i++) {

		cfg->DischargeCell1 = cellDischarge[i][0];
		cfg->DischargeCell2 = cellDischarge[i][1];
		cfg->DischargeCell3 = cellDischarge[i][2];
		cfg->DischargeCell4 = cellDischarge[i][3];
		cfg->DischargeCell7 = cellDischarge[i][4];
		cfg->DischargeCell8 = cellDischarge[i][5];
		cfg->DischargeCell9 = cellDischarge[i][6];
		cfg->DischargeCell10 = cellDischarge[i][7];

		writeConfigAddress(config, config.address[i]);

	}

	cfg->DischargeCell1 = 0;
	cfg->DischargeCell2 = 0;
	cfg->DischargeCell3 = 0;
	cfg->DischargeCell4 = 0;
	cfg->DischargeCell7 = 0;
	cfg->DischargeCell8 = 0;
	cfg->DischargeCell9 = 0;
	cfg->DischargeCell10 = 0;

	return 0;
	
}

/*bool dischargeCell(BMSconfigStructTypedef config, bool cellDischarge[8]) {
	
	BMSconfigStructTypedef *cfg;
	cfg = &config;
	
	cfg->DischargeCell1 = cellDischarge[0];
	cfg->DischargeCell2 = cellDischarge[1];
	cfg->DischargeCell3 = cellDischarge[2];
	cfg->DischargeCell4 = cellDischarge[3];
	cfg->DischargeCell7 = cellDischarge[4];
	cfg->DischargeCell8 = cellDischarge[5];
	cfg->DischargeCell9 = cellDischarge[6];
	cfg->DischargeCell10 = cellDischarge[7];
	
	writeConfig(config, 0, 1);
	
	return 0;
	
}*/

bool readRegister(CommandCodeTypedef command, uint8_t address, uint16_t *data) {
	
	uint8_t cmd[12];
	uint8_t rx_data[12];
	uint16_t PEC_return;
	uint8_t *PEC_send;
	bool dataValid = true;

	PEC_send = (uint8_t *)malloc(6*sizeof(uint8_t));
	
	PEC_send[0] = (uint8_t) (0x80 | ((address << 3) & 0x78) | ((command >> 8) & 0x07));
	PEC_send[1] = (uint8_t) (command & 0xFF);

	cmd[0] = PEC_send[0];
	cmd[1] = PEC_send[1];
	
	PEC_return = calculatePEC(2, PEC_send);
	
	cmd[2] = (PEC_return >> 8) & 0xFF;
	cmd[3] = PEC_return & 0xFF;
	
	cmd[4] = 0;
	cmd[5] = 0;
	cmd[6] = 0;
	cmd[7] = 0;
	cmd[8] = 0;
	cmd[9] = 0;
	cmd[10] = 0;
	cmd[11] = 0;
	
	SPIWriteRead(cmd, rx_data, sizeof(cmd)); // send 4 command bytes, receive 6 cell voltage bytes (4-9) and 2 PEC bytes (10-11)
	
	// calculate PEC based on cell voltage data received
	PEC_send[0] = rx_data[4]; // cell 1 voltage low bytes
	PEC_send[1] = rx_data[5]; // cell 1 voltage high bytes
	PEC_send[2] = rx_data[6]; // cell 2 voltage low bytes
	PEC_send[3] = rx_data[7]; // cell 2 voltage high bytes
	PEC_send[4] = rx_data[8]; // cell 3 voltage low bytes
	PEC_send[5] = rx_data[9]; // cell 3 voltage high bytes
	
	PEC_return = calculatePEC(6, PEC_send);
	
	// check if received PEC matches calculated PEC
	if (PEC_return != (((rx_data[10] << 8) & 0xFF00) | (rx_data[11] & 0x00FF))) {
		dataValid = false;
	}
	
	if (command == ReadCellVoltageRegisterGroup1to3) {
		
		data[0] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[1] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[2] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
		//data[3] = (uint16_t) ((rx_data[10] << 8) & 0xFF00) | (rx_data[11] & 0x00FF);
		//data[4] = PEC_return;
	}

	if (command == ReadCellVoltageRegisterGroup4to6) {
		
		data[3] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[4] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[5] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}
	
	if (command == ReadCellVoltageRegisterGroup7to9) {
		
		data[6] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[7] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[8] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}
	
	if (command == ReadCellVoltageRegisterGroup10to12) {
		
		data[9] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[10] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[11] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}
	
	if (command == ReadAuxiliaryGroupA) {
		
		data[0] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[1] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[2] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}
	
	if (command == ReadAuxiliaryGroupB) {
		data[3] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
	}
	
	if (command == ReadConfigurationRegisterGroup) {
		data[0] = (uint16_t) ((rx_data[4] << 8) & 0xFF00) | (rx_data[5] & 0x00FF);
		data[1] = (uint16_t) ((rx_data[6] << 8) & 0xFF00) | (rx_data[7] & 0x00FF);
		data[2] = (uint16_t) ((rx_data[8] << 8) & 0xFF00) | (rx_data[9] & 0x00FF);
		data[3] = (uint16_t) ((rx_data[10] << 8) & 0xFF00) | (rx_data[11] & 0x00FF);
	}
	
	return(dataValid);
	
};

void sendBroadcastCommand(CommandCodeTypedef command) {
	
	uint8_t cmd[4];
	uint16_t PEC_return;
	
	cmd[0] = (uint8_t) ((command >> 8) & 0x0F);
	cmd[1] = (uint8_t) (command & 0xFF);
	
	PEC_return = calculatePEC(2, (uint8_t *)&(cmd));
	
	cmd[2] = (PEC_return >> 8) & 0xFF;
	cmd[3] = PEC_return & 0xFF;
	
	SPIWrite(cmd, 4);
};

void sendAddressCommand(CommandCodeTypedef command, uint8_t address) {
	
	uint8_t cmd[4];
	uint16_t PEC_return;
	uint8_t *msbytes;
	
	msbytes = (uint8_t *)malloc(2*sizeof(uint8_t));
	
	msbytes[0] = (uint8_t) (0x80 | ((address << 3) & 0x78) | ((command >> 8) & 0x07));
	msbytes[1] = (uint8_t) (command & 0xFF);

	cmd[0] = msbytes[0];
	cmd[1] = msbytes[1];
	
	PEC_return = calculatePEC(2, msbytes);
	
	cmd[2] = (PEC_return >> 8) & 0xFF;
	cmd[3] = PEC_return & 0xFF;
	
	SPIWrite(cmd, 4);
	
	free(msbytes);
};

uint16_t calculatePEC(uint8_t len, uint8_t *data) {
	
	uint16_t remainder, address;
	remainder = 16; //PEC seed
	
	for (int i = 0; i < len; i++) {
		address = ((remainder >> 7) ^ data[i]) & 0xFF; //calculate PEC table address
		remainder = (remainder << 8) ^ pec15Table[address];
	}
	
	return (remainder * 2); //The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
};
