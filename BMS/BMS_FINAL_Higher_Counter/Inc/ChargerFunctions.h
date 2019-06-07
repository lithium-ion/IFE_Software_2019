#include "BMSconfig.h"


void setDischarge(BMSconfigStructTypedef cfg, uint16_t allVoltage[12][12], bool allConnection[12][12], bool cellDischarge[12][8], uint8_t chargeRate, uint16_t lowestVoltage);
uint16_t getLowestVoltage(BMSconfigStructTypedef config, uint16_t cellVoltage[12][8]);

