#include "SPI.h"
#include "BMSconfig.h"


typedef enum {
	WriteConfigurationRegisterGroup = 0x001,
	ReadCellVoltageRegisterGroup1to3 = 0x004,
	ReadCellVoltageRegisterGroup4to6 = 0x006,
	ReadCellVoltageRegisterGroup7to9 = 0x008,
	ReadCellVoltageRegisterGroup10to12 = 0x00A,
	StartCellVoltageADCConversionAll = 0x260 // MD = 00, DCP = 0, CHG = 000
	//StartCellVoltageADCConversion1and7 = 0x361; // MD = 10, DCP = 0, CHG = 001
	//StartCellVoltageADCConversion1and7 = 0x362; // MD = 10, DCP = 0, CHG = 010
	//StartCellVoltageADCConversion1and7 = 0x363; // MD = 10, DCP = 0, CHG = 011
	//StartCellVoltageADCConversion1and7 = 0x364; // MD = 10, DCP = 0, CHG = 100
} CommandCodeTypedef;

void initPECTable(void);
void writeConfig(BMSconfigStructTypedef cfg, uint8_t total_ic);
bool readCellVoltage(uint8_t address, uint16_t cellVoltage[8]);
void sendBroadcastCommand(CommandCodeTypedef command);
void sendAddressCommand(CommandCodeTypedef command, uint8_t address);
uint16_t calculatePEC(uint8_t len, uint8_t *data);