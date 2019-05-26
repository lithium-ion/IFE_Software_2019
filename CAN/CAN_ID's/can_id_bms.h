/**	
 * 	Nov 30, 2018
 * 	IFE 2018-19
 *	Chaitanya Sindagi
 *
 * 	This file contains CAN Identifiers for the BMS board
 */

// Size(bytes), Frequency(Hz), Byte1|Byte2|Byte3.....
//
// Description
//
// #define	NAME		Identifier



// 6 bytes, 96Hz, Cell number|Status Byte|Voltage H|Voltage L|Temp H|Temp L
//
// This message contains all associated data with a single cell who's number is specified in Byte1
// Status byte-
// Bit  | Description
// ---------------------------------------------------
// 7-5	| Charging state
// 4    | 1 if overtemperature 
// 3    | 1 if temp sensor is disconnected
// 2	| 1 if being balanced
// 1	| 1 if data is valid (Any data received will still be transmitted)
// 0	| 1 if cell is connected (Data fields are undefined)
// The voltage and temperature are short int (16bits) split as a High and Low byte. 
// Actual voltage     in V is obtained by  (x/10000)   min - 0,    max - 6.5535
// Actual temperature in C is obtained by  (x/500-20)  min - -20,  max - 111
// Charging states:
// 0 - pack is charging, cell is not being balanced
// 1 - pack is charging, cell is being balanced
// 2 - pack is not charging, cell is not being discharged
// 3 - pack is not charging, cell is being discharged because its voltage greatly exceeds the minimum
// 4 - pack is not charging, cell is being discharged because its voltage is greater than an absolute threshold (4.18V)
//
#define 	CELLVAL		0x007

// 6 bytes, 1Hz, Status Byte|OV cell nunmber|UV cell number|OT cell number|Disconnected voltage cell number|Disconnected temp cell number
//
// This message contains status information of the BMS as a whole
// Status byte-
// Bit  | Description
// ---------------------------------------------------
// 7-6	| Undefined
// 5    | 1 if there is a disconnected board fault
// 4    | 1 if there is a disconnected temp sensor fault
// 3    | 1 if there is a disconnected cell fault
// 2	| 1 if there is an OT fault
// 1	| 1 if there is an UV fault
// 0	| 1 if there is an OV fault
// Cell numbers are shown directly. If there is no fault, the highest/lowest cell number is shown
//
#define 	BMSSTAT		0x008

// 8 bytes, 1Hz, Voltage Max H|Voltage Max L|Cell Max number|Voltage Min H|Voltage Min L|Cell Min number|Voltage Avg H|Voltage Avg L
//
// This message contains summarized voltage data
// 
// The voltages are short int (16bits) split as a High and Low byte. 
// Actual voltage in V is obtained by  (x/10000)  min - 0,    max - 6.5535
//
#define 	BMSVINF		0x009

// 8 bytes, 1Hz, Temp Max H|Temp Max L|Cell Max number|Temp Min H|Temp Min L|Cell Min number|Temp Avg H|Temp Avg L
//
// This message contains summarized voltage data
// 
// The temperatures are short int (16bits) split as a High and Low byte. 
// Actual temperature in C is obtained by  (x/500-20)  min - -30,  max - 101
//
#define 	BMSTINF		0x00A

// 6 bytes, 1Hz, Pack Voltage H|Pack Voltage L|HR Pack Current H|HR Pack Current L|Pack Current H|Pack Current L
//
// This message contains status information for the entire battery pack
//
// The voltage is a short int (16bits) split as a High and Low byte.
// The HR current is a short int (16bits) split as a High and Low byte.
// The full-range current is a short int (16bits) split as High and Low byte.
// Actual voltage in V is obtained by (64*x/10000)  min - 0,  max - 419
// Actual HR current in A is obtained by  ((3.3*x/4096 - 2.5) * 10)  min - -20 (x=0.5), max - 8 (x=3.3)
// Actual full-range current in A is obtained by ((3.3*x/4096 - 2.5) * 250) min - -500 (x=0.5), max - 200 (x=3.3)
//
#define		BMSPACKSTAT	 0x00B
