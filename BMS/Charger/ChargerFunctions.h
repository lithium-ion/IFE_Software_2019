/************** Voltage Thresholds *******************/
extern float BMSFault_Threshold;			// max voltage (V) before BMS Fault is thrown
extern float lowerVoltage_Threshold;			// max voltage (V) before charge current is lowered
extern float higherVoltage_Threshold;		// max voltage (V) before charge current is set to 0
extern float balancingDifference;			// max difference in voltage (V) between current cell and cell with lowest voltage before current cell discharges
/************* End Voltage Thresholds ****************/


/*************** Charging Settings ********************/
/* hex value of ten times the current (A) in normal operation(when no cell is above lowerVoltage_Threshold) */
extern uint16_t normalCurrent;		// 6.2 A

/* hex value of ten times the current (A) when any cell exceeds lowerVoltage_Threshold */
extern uint16_t lowerCurrent;			// 1 A

/* hex value of ten times the voltage (V) of the charger */
extern uint16_t chargerVoltage;		// 400 V
/************** End Charging Settings *****************/


/************* Battery and Cell Numbers **************/
extern int numBoards;
extern int cellsPerBoard;
extern int cellsDischarge[12][8];			// 1 for discharging, 0 for not discharging
/************* End Battery and Cell Numbers **********/


/******************** Other **************************/
extern int chargeCurrent;		// flag used by program (2 for normal current, 1 for lower current, 0 for not charging)
extern uint8_t CANtx[8];		// CAN transmission data for the charger
/******************* End Other ***********************/

int setChargeDischarge (int cellsDischarge[numBoards][cellsPerBoard], int *chargeCurrent, float cellVoltage[numBoards][cellsPerBoard]);
float getLowestVoltage(float cellVoltage[numBoards][cellsPerBoard]);
float getLowestVoltage(float cellVoltage[numBoards][cellsPerBoard]);
void setChargerTxData();