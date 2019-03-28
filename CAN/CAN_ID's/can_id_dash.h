/**
*
*	Nick Tuczak
*
*	This is meant for all CAN ID's for the DASH BOARD
*	
**/




#define CURRENT_ID		0x00F 	//Current knob
#define TC_ID			0x010	// traction control knob
#define DRS_ID			0x011	// drs knob
#define CUSTOM			0x012	// open ended knob if we want


//ALL of these WILL FOLLOW THE SAME PROTOCOL
// HERE IS THE DEFINED PROTOCOL
// MESSAGE LENGTH = 8 bits

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
