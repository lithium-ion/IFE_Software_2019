uint16_t voltage[12][12];
uint16_t temp[12][12];
//uint16_t connection[12][12];

bool cellDisconnectFault = false;

//uint8_t BMSSTAT_data[6]; // also needs Tx header



// timing???

void main_loop(void) {
	
	while(1) {
		
		readAllCellVoltages(voltage);
		readAllCellTemps(temp);
		//cellDisconnectFault = checkAllCellConnections(connection);
		
		//have a function that determines which faults have occurred and sets the BMS fault output
			//check for overvoltage or undervoltage
			//have a function that calls getLowestVoltage and compares that to undervoltage threshold
			//call getHighestVoltage and compare that to overvoltage threshold
			//change BMSSTAT_data
	
			//if cellDisconnectFault is set, change BMSSTAT_data
	
	
	
	
}