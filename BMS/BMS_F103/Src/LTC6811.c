/*
	LTC6811 Command Functions
	Matt Vlasaty and ya boi Evan Chen
	April 18th, 2019
	
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
		writeConfig: void writeConfig(BMSconfigStructTypedef cfg, uint8_t total_ic);
			- writes configuration data from BMSconfig struct into LTC configuration register
			- BMSconfig struct includes ADCMode, UV, OV, and discharge enable for each cell
			- currently assumes that addresses are in order and start from 0 (i.e. 0, 1, 2, 3, ...) 
		readConfig: bool readConfig(uint8_t address, uint8_t cfg[6]);
			- uses general readRegister function to check current state of LTC configuration register
			- this is mostly for testing purposes
			- returns true if received PEC matches calculated PEC (same as readRegister)
	
	Set Cell Discharge Functions
		void setDischarge(bool ctd[12], BMSconfigStructTypedef *cfg, uint8_t total_ic);
			 - not tested
			 - Enables discharging on the requested cells.
			 - Parameters:
				> The cells to discharge (ctd) are stored in bool ctd[12]. True = discharge. Else don't discharge.
				  Note that the used cells may not be contiguous in order. For Apollo used cells are 1, 2, 3, 4, 7, 8, 9, 10.
				  **Subtract one from the cell number to get the corresponding array index.**
				> BMSconfigStructTypedef cfg is the original config struct used when initializing configuration registers.
				  Should be the same variable as the first writeConfig parameter in main.
				> uint8_t total_ic is the number of ICs connected. Should be the same variable as the first writeConfig parameter in main.

	Cell Voltage Functions
		readCellVoltage: bool readCellVoltage(uint8_t address, uint16_t cellVoltage[12]);
			- sends ADCV command that begins conversion for every cell to specified LTC
			- reads all cell voltage registers using general readRegister function
			- used cell inputs (1, 2, 3, 4, 7, 8, 9, 10) must be chosen from cellVoltage[12] later
			- returns true if received PEC matches calculated PEC (same as readRegister)
		readAllCellVoltages: bool readAllCellVoltages(uint16_t cellVoltage[12][12]);
			- not tested
			- stores cell voltage for every cell on every board into 2D array
			- returns false if any readCellVoltage returns false
	
	Cell Temperature Functions
		readCellTemp: bool readCellTemp(uint8_t address, uint16_t cellTemp[4]);
			- not tested
			- initiates ADC conversion for GPIO inputs connected to temperature sensors
			- reads auxiliary register groups using general readRegister function
			- converts measured voltage into temperature (need to confirm that the conversion is correct for any measurement)
			- the function that checks for faults will need to compare to temperature thresholds (not voltage)
		readAllCellTemps: bool readAllCellTemps(uint16_t cellTemp[12][4]);
			- not tested
			- stores cell temperature for every cell on every board into 2D array
			- returns false if any readCellTemp returns false
	
	Cell Connection Functions
		checkCellConnection: bool checkCellConnection(uint16_t cellVoltage[12], bool cellConnection[12]);
			- not tested
			- needs previously obtained cell voltage measurements
			- initiates ADOW command with PUP = 0 to source 100uA while measuring cell voltage
			- reads cell voltages with readCellVoltage function
			- compares previously measured values to open wire check values
			- if there is a significant drop in voltage (should be configurable), cell is disconnected
			- stores 1 into cellConnection if cell is connected
			- stores 0 into cellConnection if cell is disconnected
			- returns false if any cell is disconnected
		checkAllCellConnections: needs to be added
	
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
		- test readCellTemp, setDischarge and checkCellConnection with BMS slave board
		- resolve issue with writeConfig not changing every bit in the first register group
		- test functions used to read from every board
		- minor changes (using more user-defined constants, changing return values)
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
 
void writeConfig(BMSconfigStructTypedef cfg, uint8_t total_ic) {
	
	/*uint8_t TxBuffer[12] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC};
	uint8_t RxBuffer[12];
	
	if (cfg.ADCModeOption == 0) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	
	if (SPIWriteRead(TxBuffer, RxBuffer, 12) == 1) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);*/
	
	uint8_t config[6];
	uint8_t *cmd;
	uint8_t cmd_len = 12;
	uint16_t PEC_return;
	
	cmd = (uint8_t *)malloc(cmd_len*sizeof(uint8_t));
	
	config[0] = 0xFF;
	config[1] = 0xFF;
	//config[0] = (uint8_t) (cfg.GPIO5PulldownOff << 7) | (cfg.GPIO4PulldownOff << 6) | (cfg.GPIO3PulldownOff << 5) | (cfg.GPIO2PulldownOff << 4) | (cfg.GPIO1PulldownOff << 3) | (cfg.ReferenceOn << 2) | (cfg.ADCModeOption);
	//config[1] = (uint8_t) (cfg.UndervoltageComparisonVoltage & 0xFF);
	config[2] = (uint8_t) ((cfg.OvervoltageComparisonVoltage << 4) & 0xF0) | ((cfg.UndervoltageComparisonVoltage >> 8) & 0x0F);
	config[3] = (uint8_t) ((cfg.OvervoltageComparisonVoltage >> 4) & 0xFF);
	config[4] = (uint8_t) (cfg.DischargeCell8 << 7) | (cfg.DischargeCell7 << 6) | (cfg.DischargeCell6 << 5) | (cfg.DischargeCell5 << 4) | (cfg.DischargeCell4 << 3) | (cfg.DischargeCell3 << 2) | (cfg.DischargeCell2 << 1) | (cfg.DischargeCell1);
	config[5] = (uint8_t) ((cfg.DischargeTimeoutValue << 4) & 0xF0) | (cfg.DischargeCell12 << 3) | (cfg.DischargeCell11 << 2) | (cfg.DischargeCell10 << 1) | (cfg.DischargeCell9);
	
	for (uint8_t current_ic = 0; current_ic <= total_ic-1; current_ic++) {
	
		cmd[0] = (uint8_t) (0x80 | ((current_ic << 3) & 0x78) | ((WriteConfigurationRegisterGroup >> 8) & 0x07));
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
		
		//HAL_Delay(100);
	}
	
	free(cmd);
	
};

/*
 - Enables discharging on the requested cells.
 - Parameters:
	> The cells to discharge (ctd) are stored in bool ctd[12]. True = discharge. Else don't discharge.
	  Note that the used cells may not be contiguous in order. For Apollo 18-19, used cells are 1, 2, 3, 4, 7, 8, 9, 10.
	> BMSconfigStructTypedef cfg is the original config struct used when initializing configuration registers.
	  Should be the same variable as the first writeConfig parameter in main.
	> uint8_t total_ic is the number of ICs connected. Should be the same variable as the first writeConfig parameter in main.
*/
void setDischarge(bool ctd[12], BMSconfigStructTypedef *cfg, uint8_t total_ic) {
	if (ctd[0] == true) { cfg->DischargeCell1 = 1; }
	else				{ cfg->DischargeCell1 = 0; }
	
	if (ctd[1] == true) { cfg->DischargeCell2 = 1; }
	else				{ cfg->DischargeCell2 = 0; }

	if (ctd[2] == true) { cfg->DischargeCell3 = 1; }
	else				{ cfg->DischargeCell3 = 0; }
	
	if (ctd[3] == true) { cfg->DischargeCell4 = 1; }
	else				{ cfg->DischargeCell4 = 0; }
	
	if (ctd[4] == true) { cfg->DischargeCell5 = 1; }
	else				{ cfg->DischargeCell5 = 0; }

	if (ctd[5] == true) { cfg->DischargeCell6 = 1; }
	else				{ cfg->DischargeCell6 = 0; }

	if (ctd[6] == true) { cfg->DischargeCell7 = 1; }
	else				{ cfg->DischargeCell7 = 0; }
	
	if (ctd[7] == true) { cfg->DischargeCell8 = 1; }
	else				{ cfg->DischargeCell8 = 0; }

	if (ctd[8] == true) { cfg->DischargeCell9 = 1; }
	else				{ cfg->DischargeCell9 = 0; }

	if (ctd[9] == true) { cfg->DischargeCell10 = 1; }
	else				{ cfg->DischargeCell10 = 0; }
	
	if (ctd[10] == true) { cfg->DischargeCell11 = 1; }
	else				 { cfg->DischargeCell11 = 0; }

	if (ctd[11] == true) { cfg->DischargeCell12 = 1; }
	else				 { cfg->DischargeCell12 = 0; }
		
	writeConfig(*cfg, total_ic);
}

bool readCellVoltage(uint8_t address, uint16_t cellVoltage[12]) {
	
	bool PEC_check = false;
	bool dataValid = true;
	uint16_t *voltage;
	
	voltage = (uint16_t *)malloc(12*sizeof(uint16_t));
	
	sendAddressCommand(StartCellVoltageADCConversionAll, address); // start conversion for every cell

	HAL_Delay(15); // conversion time is 12.8ms at 422Hz, so wait 15ms
	
	PEC_check = readRegister(ReadCellVoltageRegisterGroup1to3, address, voltage);
	dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadCellVoltageRegisterGroup4to6, address, voltage);
	dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadCellVoltageRegisterGroup7to9, address, voltage);
	dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadCellVoltageRegisterGroup10to12, address, voltage);
	dataValid = dataValid & PEC_check;
	
	for (uint8_t i = 0; i < 12; i++) {
		cellVoltage[i] = voltage[i];
	}
	
	free(voltage);
	
	return(dataValid);
	
};

bool readAllCellVoltages(uint16_t cellVoltage[12][12]) {
	
	uint16_t boardVoltage[12];
	
	for (uint8_t i = 0; i < 12; i++) {
		
		readCellVoltage(i, boardVoltage);
		for (uint8_t j = 0; i < 12; i++) {
			cellVoltage[i][j] = boardVoltage[j];
		}
	}
	
	return 0; // should return false if any readCellVoltage returns false
}

bool readCellTemp(uint8_t address, uint16_t cellTemp[4]) {
	
	bool PEC_check = false;
	bool dataValid = true;
	uint16_t *temp;
	double dummy[4];
	double conversion[4]; 
	
	temp = (uint16_t *)malloc(4*sizeof(uint16_t));
	
	sendAddressCommand(StartCellTempVoltageADCConversionAll, address);
	
	HAL_Delay(15); // conversion time is 12.8ms at 422Hz, so wait 15ms
	
	//readRegister(command, address, *data)
	PEC_check = readRegister(ReadAuxiliaryGroupA, address, temp);
	dataValid = dataValid & PEC_check;
	PEC_check = readRegister(ReadAuxiliaryGroupB, address, temp);
	dataValid = dataValid & PEC_check;
	
	for (uint8_t i = 0; i < 4; i++) {
		dummy[i] = (float) temp[i] / 10000;
		conversion[i] = 37735 - 113923 * dummy[i] + 142663 * pow(dummy[i], 2) - 94465 * pow(dummy[i], 3) + 34799 * pow(dummy[i], 4) - 6749 * pow(dummy[i], 5) + 537.11 * pow(dummy[i], 6);
		cellTemp[i] = (uint16_t) conversion[i];
		cellTemp[i] = 500 * (cellTemp[i] + 20);
	}
	
	free(temp);
	
	return (dataValid);
	
};

bool readAllCellTemps(uint16_t cellTemp[12][4]) {
	
	uint16_t boardTemp[4];
	
	for (uint8_t i = 0; i < 12; i++) {
		
		readCellTemp(i, boardTemp);
		for (uint8_t j = 0; j < 4; j++) {
			cellTemp[i][j] = boardTemp[j];
		}
	}
	
	return 0; // should return false if any readCellTemp returns false
}

bool readConfig(uint8_t address, uint8_t cfg[6]) {
	
	uint16_t *config;
	bool dataValid = false;
	
	config = (uint16_t *)malloc(3*sizeof(uint16_t));
	
	dataValid = readRegister(ReadConfigurationRegisterGroup, address, config);
	
	cfg[0] = (uint8_t) ((config[0] >> 8) & 0xFF);
	cfg[1] = (uint8_t) (config[0] & 0xFF);
	cfg[2] = (uint8_t) ((config[1] >> 8) & 0xFF);
	cfg[3] = (uint8_t) (config[1] & 0xFF);
	cfg[4] = (uint8_t) ((config[2] >> 8) & 0xFF);
	cfg[5] = (uint8_t) (config[2] & 0xFF);
	
	return dataValid;
}

bool checkCellConnection(uint16_t cellVoltage[12], bool cellConnection[12]) {
		
	uint16_t ADOWvoltage[12];
	
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	sendBroadcastCommand(StartOpenWireConversionPulldown);
	
	HAL_Delay(15);
	
	readCellVoltage(0, ADOWvoltage);
	
	for (uint8_t i = 0; i < 12; i++) {
		// if voltage fell by > 100mV
		if ((ADOWvoltage[i] - cellVoltage[i]) > 1000) {
			cellConnection[i] = 0;
		}
		else {
			cellConnection[i] = 1;
		}
	}

	return 0; // should return false if any cell is disconnected 
}

bool readRegister(CommandCodeTypedef command, uint8_t address, uint16_t *data) {
	
	//send command and address in first two bytes
	//send PEC in next two bytes
	//SPI writeread
	//calculate PEC based off received data bytes
	//if received PEC matches calculated PEC, return true (data valid)
	
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
	}

	if (command == ReadCellVoltageRegisterGroup1to3) {
		
		data[3] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[4] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[5] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}
	
	if (command == ReadCellVoltageRegisterGroup1to3) {
		
		data[6] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF);
		data[7] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF);
		data[8] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF);
	}
	
	if (command == ReadCellVoltageRegisterGroup1to3) {
		
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
