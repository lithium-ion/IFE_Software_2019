#include "SPI.h"
#include "BMSconfig.h"
#include "math.h"

typedef enum {
	WriteConfigurationRegisterGroup = 0x001,
	ReadConfigurationRegisterGroup = 0x002,
	ReadCellVoltageRegisterGroup1to3 = 0x004,
	ReadCellVoltageRegisterGroup4to6 = 0x006,
	ReadCellVoltageRegisterGroup7to9 = 0x008,
	ReadCellVoltageRegisterGroup10to12 = 0x00A,
	ReadAuxiliaryGroupA = 0x00C,
	ReadAuxiliaryGroupB = 0x00E,
	StartOpenWireConversionPulldown = 0x229,
	StartCellVoltageADCConversionAll = 0x260, // MD = 00, DCP = 0, CHG = 000
	StartCellTempVoltageADCConversionAll = 0x460 // MD = 00, CHG = 000
} CommandCodeTypedef;

void initPECTable(void);
void writeConfig(BMSconfigStructTypedef cfg, uint8_t total_ic);
//bool readCellVoltage(uint8_t address, uint16_t cellVoltage[8]);
void setDischarge(bool ctd[12], BMSconfigStructTypedef *cfg, uint8_t total_ic);
bool readCellVoltage(uint8_t address, uint16_t cellVoltage[12]);
bool readAllCellVoltages(uint16_t cellVoltage[12][12]);
bool readCellTemp(uint8_t address, uint16_t cellTemp[4]);
bool readAllCellTemps(uint16_t cellTemp[12][4]);
bool checkCellConnection(uint16_t cellVoltage[12], bool cellConnection[12]);
bool readConfig(uint8_t address, uint8_t cfg[6]);
bool readRegister(CommandCodeTypedef command, uint8_t address, uint16_t *data);
void sendBroadcastCommand(CommandCodeTypedef command);
void sendAddressCommand(CommandCodeTypedef command, uint8_t address);
uint16_t calculatePEC(uint8_t len, uint8_t *data);