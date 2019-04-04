/**
*
*	Nick Tuczak
*
*	This is meant for all CAN ID's for the DASH BOARD
*	
**/

#define DASH_CAN_ID		0x00F

// MESSAGE LENGTH: 4 bytes
// BYTE 1: CURRENT_POT VALUE
// BYTE 2: CUSTOM_POT VALUE
// BYTE 3: TC_POT VALUE
// BYTE 4: DRS_POT VALUE

// EACH BYTE FOLLOWS THIS PROTOCOL

// (Base 10) | (Binary)   | Description
//============================
// 		0	 |	 0000     |	Knob turned to value 1 (Is currently not in use)
// 		1	 |	 0001     |	knob turned to value 2
// 		2	 |	 0010     | knob turned to value 3
//		3	 |	 0011     | knob turned to value 4
// 		4	 |	 0100     | knob turned to value 5
// 		5	 |	 0101     | knob turned to value 6
// 		6	 |	 0110     | knob turned to value 7
// 		7	 |	 0111     | knob turned to value 8
// 		8	 |	 1000     | knob turned to value 9
// 		9	 |	 1001     | knob turned to value 10
//
//**** I understand that means the can value is one off from
//**** the knob position. We also zero index so get over it
//
//
//
//
//
