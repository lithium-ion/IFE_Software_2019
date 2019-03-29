/*
*
* Nicholas Tuczak
* CAN ID's for the Safety Board
*
*
*
*/




#define FAULTS 0x0D0
/*
*
* SIZE = 3 Bytes 
*  Byte # 	|	Type 	| Description 			| Name
------------------------------------------------------
* 	0 		| 	Boolean	| 	0xFF = Fault present| BMS
*			|			| 	0x00 = no FAULT 	|
-----------------------------------------------------
*	1		| 	Boolean	|	0xFF = Fault present| IMD 
*			| 			|	0x00 = no fault 	|
------------------------------------------------------
* 	2 		| 	Boolean	|	0xFF = Fault present| BSPD
*			|			| 	0x00 = no FAULT 	| 
------------------------------------------------------
* 	3 		| 	Boolean	|	0xFF = Fault present| APPS
*			|			| 	0x00 = no FAULT 	| 
*
*/

#define PRECHARGE 0x0D1
/*
*
* Size = 1 Byte 
*	Byte 	| Type 		| Description 			| Name
------------------------------------------------------
*	0		|	Boolean	|	0xFF = Charged		| Precharge
			|			|	0x00 = not charged 	|	
*
*
*/

#define ENABLE	0x0D2
/*
*
* Size = 1 Byte 
*	Byte 	| Type 		| Description 			| Name
------------------------------------------------------
*	0		|	Boolean	|	0xFF = Enabled 		| Enabled
			|			|	0x00 = not Enabled 	|	
*
*
*/
