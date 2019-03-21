#include "LTC6811.h"

//CommandCodeTypedef CommandCode;

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
	
	config[0] = (uint8_t) (cfg.GPIO5PulldownOff << 7) | (cfg.GPIO4PulldownOff << 6) | (cfg.GPIO3PulldownOff << 5) | (cfg.GPIO2PulldownOff << 4) | (cfg.GPIO1PulldownOff << 3) | (cfg.ReferenceOn << 2) | (cfg.ADCModeOption);
	config[1] = (uint8_t) (cfg.UndervoltageComparisonVoltage & 0xFF);
	config[2] = (uint8_t) ((cfg.OvervoltageComparisonVoltage << 4) & 0xF0) | ((cfg.UndervoltageComparisonVoltage >> 8) & 0x0F);
	config[3] = (uint8_t) ((cfg.OvervoltageComparisonVoltage >> 4) & 0xFF);
	config[4] = (uint8_t) (cfg.DischargeCell8 << 7) | (cfg.DischargeCell7 << 6) | (cfg.DischargeCell6 << 5) | (cfg.DischargeCell5 << 4) | (cfg.DischargeCell4 << 3) | (cfg.DischargeCell3 << 2) | (cfg.DischargeCell2 << 1) | (cfg.DischargeCell1);
	config[5] = (uint8_t) ((cfg.DischargeTimeoutValue << 4) & 0xF0) | (cfg.DischargeCell12 << 3) | (cfg.DischargeCell11 << 2) | (cfg.DischargeCell10 << 1) | (cfg.DischargeCell9);
	
	for (uint8_t current_ic = 1; current_ic <= total_ic; current_ic++) {
	
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
		
		HAL_Delay(100);
	}
	
	free(cmd);
	
};

bool readCellVoltage(uint8_t address, uint16_t cellVoltage[8]) {
	
	uint8_t cmd[12];
	uint8_t rx_data[12];
	uint16_t PEC_return;
	uint8_t *PEC_send;
	bool dataValid = true;
	
	sendAddressCommand(StartCellVoltageADCConversionAll, address); // start conversion for every cell

	HAL_Delay(15); // conversion time is 12.8ms at 422Hz, so wait 15ms

	// group 1: cells 1, 2, and 3
	PEC_send = (uint8_t *)malloc(6*sizeof(uint8_t));
	
	PEC_send[0] = (uint8_t) (0x80 | ((address << 3) & 0x78) | ((ReadCellVoltageRegisterGroup1to3 >> 8) & 0x07));
	PEC_send[1] = (uint8_t) (ReadCellVoltageRegisterGroup1to3 & 0xFF);

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
	
	/*cellVoltage[0] = ((rx_data[0] << 8) & 0xFF00) | (rx_data[1] & 0x00FF);
	cellVoltage[1] = ((rx_data[2] << 8) & 0xFF00) | (rx_data[3] & 0x00FF);
	cellVoltage[2] = ((rx_data[4] << 8) & 0xFF00) | (rx_data[5] & 0x00FF);
	cellVoltage[3] = ((rx_data[6] << 8) & 0xFF00) | (rx_data[7] & 0x00FF);
	cellVoltage[4] = ((rx_data[8] << 8) & 0xFF00) | (rx_data[9] & 0x00FF);
	cellVoltage[5] = ((rx_data[10] << 8) & 0xFF00) | (rx_data[11] & 0x00FF);*/
	
	// calculate PEC based on cell voltage data received
	PEC_send[0] = rx_data[4]; // cell 1 voltage low bytes
	PEC_send[1] = rx_data[5]; // cell 1 voltage high bytes
	PEC_send[2] = rx_data[6]; // cell 2 voltage low bytes
	PEC_send[3] = rx_data[7]; // cell 2 voltage high bytes
	PEC_send[4] = rx_data[8]; // cell 3 voltage low bytes
	PEC_send[5] = rx_data[9]; // cell 3 voltage high bytes
	
	PEC_return = calculatePEC(6, PEC_send);
	
	// check if received PEC matches calculated PEC
	/*if (PEC_return != (((rx_data[10] >> 8) & 0xFF00) | (rx_data[11] & 0x00FF))) {
		dataValid = false;
	}*/
	
	// store values if data is valid
	if (dataValid == true) {
		cellVoltage[0] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF); // cell 1 voltage
		cellVoltage[1] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF); // cell 2 voltage
		cellVoltage[2] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF); // cell 3 voltage
	}
	
	// group 2: cells 4, 5, and 6
	PEC_send[0] = (uint8_t) (0x80 | ((address << 3) & 0x78) | ((ReadCellVoltageRegisterGroup4to6 >> 8) & 0x07));
	PEC_send[1] = (uint8_t) (ReadCellVoltageRegisterGroup4to6 & 0xFF);

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
	
	SPIWriteRead(cmd, rx_data, sizeof(cmd));
	
	// calculate PEC
	PEC_send[0] = rx_data[4]; // cell 4 voltage low bytes
	PEC_send[1] = rx_data[5]; // cell 4 voltage high bytes
	PEC_send[2] = rx_data[6]; // cell 5 voltage low bytes
	PEC_send[3] = rx_data[7]; // cell 5 voltage high bytes
	PEC_send[4] = rx_data[8]; // cell 6 voltage low bytes
	PEC_send[5] = rx_data[9]; // cell 6 voltage high bytes
	
	PEC_return = calculatePEC(6, PEC_send);
	
	// store values if data is valid
	
	/*if (PEC_return != (((rx_data[10] >> 8) & 0xFF00) | (rx_data[11] & 0x00FF))) {
		dataValid = false;
	}*/
	
	if (dataValid == true) {
		cellVoltage[3] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF); // cell 4 voltage
	}
	
	// group 3: cells 7, 8, and 9
	PEC_send[0] = (uint8_t) (0x80 | ((address << 3) & 0x78) | ((ReadCellVoltageRegisterGroup7to9 >> 8) & 0x07));
	PEC_send[1] = (uint8_t) (ReadCellVoltageRegisterGroup7to9 & 0xFF);

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
	
	SPIWriteRead(cmd, rx_data, sizeof(cmd));
	
	// calculate PEC
	PEC_send[0] = rx_data[4]; // cell 7 voltage low bytes
	PEC_send[1] = rx_data[5]; // cell 7 voltage high bytes
	PEC_send[2] = rx_data[6]; // cell 8 voltage low bytes
	PEC_send[3] = rx_data[7]; // cell 8 voltage high bytes
	PEC_send[4] = rx_data[8]; // cell 9 voltage low bytes
	PEC_send[5] = rx_data[9]; // cell 9 voltage high bytes
	
	PEC_return = calculatePEC(6, PEC_send);
	
	// store values if data is valid
	
	/*if (PEC_return != (((rx_data[10] >> 8) & 0xFF00) | (rx_data[11] & 0x00FF))) {
		dataValid = false;
	}*/
	
	if (dataValid == true) {
		cellVoltage[4] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF); // cell 7 voltage
		cellVoltage[5] = (uint16_t) ((rx_data[7] << 8) & 0xFF00) | (rx_data[6] & 0x00FF); // cell 8 voltage
		cellVoltage[6] = (uint16_t) ((rx_data[9] << 8) & 0xFF00) | (rx_data[8] & 0x00FF); // cell 9 voltage
	}
	
	// group 4: cells 10, 11, and 12
	PEC_send[0] = (uint8_t) (0x80 | ((address << 3) & 0x78) | ((ReadCellVoltageRegisterGroup10to12 >> 8) & 0x07));
	PEC_send[1] = (uint8_t) (ReadCellVoltageRegisterGroup10to12 & 0xFF);

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
	
	SPIWriteRead(cmd, rx_data, sizeof(cmd));
	
	// calculate PEC
	PEC_send[0] = rx_data[4]; // cell 10 voltage low bytes
	PEC_send[1] = rx_data[5]; // cell 10 voltage high bytes
	PEC_send[2] = rx_data[6]; // cell 11 voltage low bytes
	PEC_send[3] = rx_data[7]; // cell 11 voltage high bytes
	PEC_send[4] = rx_data[8]; // cell 12 voltage low bytes
	PEC_send[5] = rx_data[9]; // cell 12 voltage high bytes
	
	PEC_return = calculatePEC(6, PEC_send);
	
	// store values if data is valid
	
	/*if (PEC_return != (((rx_data[10] >> 8) & 0xFF00) | (rx_data[11] & 0x00FF))) {
		dataValid = false;
	}*/
	
	if (dataValid == true) {
		cellVoltage[7] = (uint16_t) ((rx_data[5] << 8) & 0xFF00) | (rx_data[4] & 0x00FF); // cell 7 voltage
	}
	
	free(PEC_send);
	
	return (dataValid == true);
	
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
