/*
*
* Nichoals Tuczak
* CAN ID documentation for rienhart
* all the info here can be found via this link:
* https://app.box.com/s/4fb49r9p6lzfz4uwcb5izkxpcwh768vc
*
*/


//CAN ID OFFSET:
/*
This parameter allows the user to choose their
 own set of contiguous CAN message identifiers
 starting with the value in CAN ID Offset. This
 offset covers a range of 0 –0x7C0. The default
 offset is 0x0A0. Thus the default range is 0x0A0 –0x0CF.
 */

//Address										CAN ACTIVE UW  | CAN ACTIVE LW 

#define Tempratures1 				        0x0A0//			0x0000     |  0x0001 	
#define Tempratures2 				        0x0A1//			0x0000     |  0x0002 
#define Analog_Input_Voltages		    0x0A2//			0x0000     |  0x0008 
#define Digital_Input_Status		    0x0A3//			0x0000     |  0x0010
#define Motor_Position_information	0x0A4//			0x0000     |  0x0020 
#define Current_Information 		    0x0A6//			0x0000     |  0x0040 
#define Voltage_Information			    0x0A7//			0x0000     |  0x0080 
#define Flux_Information 			      0x0A8//			0x0000     |  0x0100  
#define Internal_Voltages 			    0x0A9//			0x0000     |  0x0200  
#define Interntal_states 			      0x0AA//			0x0000     |  0x0400  
#define Fault_Cods 					        0x0AB//			0x0000     |  0x0800 
#define Torque_Timer_Information	  0x0AC//			0x0000     |  0x1000
#define Modulation_Index_Info		    0x0AC// 		0x0000     |  0x2000
#define Flux_Weakinging_Output		  0x0AC// 		0x0000     |  0x2000
#define Firmware_Information        0x0AE// 		0x0000     |  0x4000

//**As anexample, in order to disable Temperature #1, #2and #3 
//messagesin the above table, the parameter command messageshould
// be configuredas follows:
/*
 HIGH BYTE | LOW BYTE | HIGH BYTE | LOW BYTE | DATA B3 | DATA B2 | DATA B1 | DATA B0
  CAN ACTIVE HIGH WORD| CAN ACTIVE LOW WORD  | RESERVED| R/W COM |   PARAM ADDRESSS
  	255	   |	255	  |	255		  |	248		 |    0	   |	1	 |	0		|	148
  	0Xff   |    0XFF  |  0XFF     | 0XF8
  	Data Byte 4controlsthe following messages
  	:Bit 0: Temperature #1
  	Bit 1: Temperature #2
  	Bit 2: Temperature #3
  	Bit 3: Analog Input Voltages
  	Bit 4: Digital Input Status
  	Bit 5: Motor Position Information
  	Bit 6: Current Information
  	Bit 7: Voltage InformationIn 
  	little-endian format, Byte 4 can be looked at 
  	as: Bit 7 -Bit 6 -Bit 5-... -Bit 1 -Bit 0To enable 
  	all messages above, Byte 4 should be set to 
  	0xFF (all bits set to 1).To disable temperature messages,
  	 Byte 4 should be set to 0xF8 (Bit 0, 1, and 2 are set to 0)
  	 To disable Motor position information, Byte 4 should be setto
  	  0xDF (Bit 5 set to 0)Data Byte 5controls the following messages
  	  Bit 0: Flux Information
  	  Bit 1: Internal Voltages
  	  Bit 2: Internal States
  	  Bit 3: Fault Codes
  	  Bit 4: Torque & Timer Information
  	  Bit 5: Not used
  	  Bit 6: Not used
  	  Bit 7: Not used
*/


#define R_W_to_MC		0x0C1 /*
Byte 	#Name				Format			Description
0,1		Parameter Address 	Unsigned int 	Each command is identified by a unique address 
2R/W 	command 			Boolean			0 = read, 1 = write
3		Reserved 			NA 				NA
4,5		Data          See “DataFormats”     Data should be entered as dictated in “Data Formats” section.
6,7 	Reserved 			NA 				NA
*/

#define R_W_Resp_from_MC 0x0C2
/*BYTE 	 #Name 				     Format 			       Description
0,1		Parameter address	Unsigned int 	       Will return 0,0 if parameter address is not recognized.
2     Write Success     Boolean               0 = not written, 1 = success
3      Reservedc        NAc                 NA
4,5    Data           See “Data Formats” sectionResponse data is in the format dictated in “Data Formats” section
6,7    Reserved         NA                    NA

*/

/*2.3.4.3CANConfiguration
Address    Name                            Format             Description
141       CAN ID Offset                    Unsigned integer   This parameter allows the user to choose their own set of contiguous CAN message identifiers starting with the value in CAN ID Offset.This offset covers a range of 0 –0x7C0. The default offset is 0x0A0. The default range is 0x0A0 –0x0CF. This feature is especially useful when there are more than one controlleron the same CAN network. While setting base address for a controller, it must be made sure that the address range for controllers does not contain overlapping addresses.
144       CAN Extended Message Identifier  Boolean            This parameter allows switchingbetween CAN standard and extended message identifiers.0 = Standard CAN Messages1 = Extended CAN Messages
171       CAN J1939 Option Active          Boolean            This parameter allows switching between extended message identifiers with or without SAE J1939 format.0 = J1939 formatting is not active1 = J1939 formatting is active
145       CAN Term Resistor Present       Boolean             In order to use CAN communication,theCAN busneedsto be terminated with a 120 Ohm resistor. RMS PMunits are equipped with thisresistor which is activated through this parameter.0 = Term. Resistor not active1 = Term. Resistor active(Default)If CAN Terminator Resistor isdeactivated, it may be necessary to use theGUI interface only11since CAN communication mayfail without a terminator resistor.The RM inverters do not have this feature and thus this parameter has no effect on the CAN bus. CAN Protocol40of 59
146       CAN Command Message Active       Boolean            RMS CAN requires a “heartbeat” command message.  This command message controls the inverter, motor direction, and torque or speed. In the absence (time set by CAN TimeOut parameter) of a regular broadcast of this message, The controllerwill assume there is a problem and will flag a fault unless the fault has been deactivated by setting this parameter to 0.0 = The command message 0xC0 is notsent everyhalf a second.1 = The command message 0xC0 is sent every half a second. (Default)
147       CAN Bit Rate                     Unsigned integer   250Kbps is the default bit rate. Bus speed can be changed using CAN parameter command message. However, changing this parameter requires a power reset on controllersince bus speed is setup only at the initialization of CAN modules in the microcontroller. Also, This input is restricted to valid baud rates. The 4 options for valid baud rate are:125= 125Kbps250  = 250 Kbps (Default)500= 500 Kbps1000= 1Mbps
148       CAN Active Messages Word        Unsigned long integer (32-bits)This parameter is used to enable/disable CAN Broadcast Messages. Each bit represents a CAN Message broadcast status as follows:0 = CAN Messages broadcast disabled1 = CAN Message broadcast enabled     (Default)Please refer to the table of CAN Broadcast Messages in section 2.1 for details on how to enable/disable each message.
158       CAN Diagnostic Data Transmit Active Boolean         This parameter is used to enable/disable the broadcast of the diagnosticdata.0 = CAN Diagnostic Data broadcast disabled1 = CAN Diagnostic Data broadcast enabled (Default)Please refer to the document, CAN Diagnostic Data, for more details on this feature.
159     CAN Inverter Enable Switch Active   Boolean           1 = DIN1   digital input is taken into consideration and the inverter will only be enabled if both DIN1 and inverter command are active. If either one is inactive, the inverter will be disabled. 0 = DIN1 will have no effect on enabling or disabling the inverter (Default)
172     CAN timeout                         Unsigned integer   This parameter sets how long before the CAN timeout error is set.  The timeout is only active if the CAN Command Message Activeis set to 1.  The time is set in counts of 3ms.  So for example setting a value of 333 will give a timeout time of 1 second.
177      CAN OBD2 Enable                 Boolean                0 = OBD2        Support is disabled1 –7 = OBD2 Support is enabled with the address offset defined by the value.178CAN BMS Limit EnableBoolean0 = BMS CAN Message Torque Limiting is disabled.1 = BMS CAN Message Torque Limiting is enabled.
233     CAN Slave Cmd ID                  Unsigned Integer     0 = disable slave mode0x22 thru 0x7FD enables Slave mode CAN message output.
234     CAN Slave Dir                     Unsigned Integer     0 = Direction command of Slave controller is the same as the Master.1 = Direction command of Slave controller is the opposite of the Master.