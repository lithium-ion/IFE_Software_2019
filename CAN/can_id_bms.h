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
// 7-3	| Undefined
// 2	| 1 if being balanced
// 1	| 1 if data is valid (Any data received will still be transmitted)
// 0	| 1 if cell is connected (Data fields are undefined)
// The voltage and temperature are short int (16bits) split as a High and Low byte. 
// Actual voltage     in V is obtained by  (x/10000)   min - 0,    max - 6.5535
// Actual temperature in C is obtained by  (x/500-20)  min - -20,  max - 111
//
#define 	CELLVAL		0x007

// 1 byte, 1Hz, Status Byte|OV cell nunmber|UV cell number|OT cell number
//
// This message contains status information of the BMS as a whole
// Status byte-
// Bit  | Description
// ---------------------------------------------------
// 7-5	| Undefined
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

