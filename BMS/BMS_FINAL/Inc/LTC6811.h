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
void writeConfigAddress(BMSconfigStructTypedef cfg, uint8_t address);
void writeConfigAll(BMSconfigStructTypedef cfg);
bool readCellVoltage(uint8_t address, uint16_t cellVoltage[12]);
bool readAllCellVoltages(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12]);
bool readCellTemp(uint8_t address, uint16_t cellTemp[4], bool dcFault[4], bool tempFault[4]);
bool readAllCellTemps(BMSconfigStructTypedef cfg, uint16_t cellTemp[12][4], bool dcFault[12][4], bool tempFault[12][4]);
bool checkCellConnection(uint16_t cellVoltage[12], bool cellConnection[12]);
bool checkAllCellConnections(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12], bool cellConnection[12][12], uint8_t dcCell);
bool dischargeCellGroups(BMSconfigStructTypedef cfg, bool cellDischarge[12][8]);
//bool dischargeCell(BMSconfigStructTypedef config, bool cellDischarge[8]);
bool readConfig(uint8_t address, uint8_t cfg[8]);
bool readRegister(CommandCodeTypedef command, uint8_t address, uint16_t *data);
void sendBroadcastCommand(CommandCodeTypedef command);
void sendAddressCommand(CommandCodeTypedef command, uint8_t address);
uint16_t calculatePEC(uint8_t len, uint8_t *data);