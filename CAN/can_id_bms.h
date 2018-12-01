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



// 6, 96, Cell number|Status Byte|Voltage H|Voltage L|Temp H|Temp L
//
// This message contains all associated data with a single cell who's number is specified in Byte1
// Status byte-
// Bytes| Description
// ---------------------------------------------------
// 7-3	| Undefined
// 2	| 1 if being balanced
// 1	| 1 if data is valid (Any data received will still be transmitted)
// 0	| 1 if cell is connected (Data fields are undefined)
// The voltage and temperature are short int (16bits) split as a High and Low byte. 
// Actual voltage     in V is obtained by  /10000.  min - 0,    max - 6.5535
// Actual temperature in C is obtained by  /500-20  min - -20,  max - 111
//
#define 	CELLVAL		0x07
