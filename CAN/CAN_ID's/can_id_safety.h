/*
*
* Nicholas Tuczak
* CAN ID's for the Safety Board
*
*
*
*/




#define SOFT_FAULTS 0x0D0
/*
*
* SIZE = 6 Bytes 
*  Byte # 	|	Type 	| Description 			| Name
------------------------------------------------------
* 	0 		|  upper	| Brake pressue 1 upper | BP1
*			|  			| bits					|
-----------------------------------------------------
*	1		|  lower 	| Brake pressure 1 lower|
*			|  			| bits					|
------------------------------------------------------
* 	2 		|  upper	| Brake pressue 2 upper | BP2
*			|  			| bits					|
-----------------------------------------------------
*	3		|  lower 	| Brake pressure 2 lower|
*			|  			| bits					|
------------------------------------------------------
* 	4 		|  upper	| throttle 1 upper 		| TH1
*			|  			| bits					|
-----------------------------------------------------
*	5		|  lower 	| Throttle 1 lower		|
*			|  			| bits					|
*/

#define CAR_STATE 0x00E
/*
*
* SIZE = 4 Bytes 
*  Byte # 	|	Type 	| Description 			| Name
------------------------------------------------------
* 	0 		| 	STATE	| 	LV_ON           0x01| CAR_STATE
*			|			| 	PRECHARGED      0x02|
*			|			|	ENABLE_FLIPPED  0x04|
*			|			|	RTDS_SOUND      0x08|
*			|			|	PWR_AVAILABLE   0x10|
*			|			|	SOFT_FAULT      0x20|
-----------------------------------------------------
*	1		| 	Boolean	|	0xFF = Fault present| BMS
*			| 			|	0x00 = no fault 	|
------------------------------------------------------
* 	2 		| 	Boolean	|	0xFF = Fault present| IMD
*			|			| 	0x00 = no FAULT 	| 
------------------------------------------------------
* 	3 		| 	Boolean	|	0xFF = Fault present| BSPD
*			|			| 	0x00 = no FAULT 	| 
*
*/
