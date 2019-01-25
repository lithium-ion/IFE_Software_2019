/*!
DC2350
LTC6813-1: Battery stack monitor

REVISION HISTORY
$Revision: 1200 $
$Date: 2017-01-26

@verbatim

NOTES
 Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.
   Ensure all jumpers on the demo board are installed in their default positions from the factory.
   Refer to Demo Manual DC2350.

USER INPUT DATA FORMAT:
 decimal : 1024
 hex     : 0x400
 octal   : 02000  (leading 0)
 binary  : B10000000000
 float   : 1024.0
@endverbatim


Copyright (c) 2017, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2017 Linear Technology Corp. (LTC)
 */


/*! @file
    @ingroup LTC6813-1
*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6813.h"
#include <SPI.h>

#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

void print_menu();
void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void print_stat();
void print_open();
void print_config();
void print_rxconfig();
void print_comm();
void print_rxcomm();
void print_pwm();
void print_rxpwm();
void print_sctrl();
void print_rxsctrl();
void print_pwm_sctrlb();
void print_rxpsb(); 
void print_statsoc();
void print_aux1();
void check_error(int error);
void print_pec();
char get_char();


/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/
const uint8_t TOTAL_IC = 1;//!<number of ICs in the daisy chain

//ADC Command Configurations---------------------See LTC681x.h for options
const uint8_t ADC_OPT = ADC_OPT_DISABLED;
const uint8_t ADC_CONVERSION_MODE =MD_7KHZ_3KHZ;
const uint8_t ADC_DCP = DCP_DISABLED;
const uint8_t CELL_CH_TO_CONVERT =CELL_CH_ALL;
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;
const uint8_t NO_OF_REG = REG_ALL;

const uint16_t MEASUREMENT_LOOP_TIME = 500;//milliseconds(mS)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; // Over voltage threshold ADC Code. LSB = 0.0001
const uint16_t UV_THRESHOLD = 33000; // Under voltage threshold ADC Code. LSB = 0.0001

//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = ENABLED; // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_STAT = DISABLED; //This is ENABLED or DISABLED
const uint8_t PRINT_PEC = ENABLED; //This is ENABLED or DISABLED
/************************************
  END SETUP
*************************************/

/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the LTC6813
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/

cell_asic bms_ic[TOTAL_IC];

const uint8_t PWMREG=0;
const uint8_t STREG=0; 

//Set the configuration bits

bool REFON = true;
bool ADCOPT = false;
bool gpioBits_a[5] = {false,false,true,true,true}; // Gpio 1,2,3,4,5
uint16_t UV=UV_THRESHOLD;
uint16_t OV=OV_THRESHOLD;
bool dccBits_a[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool dctoBits[4] = {false,false,false,false}; //Dcto 0,1,2,3
bool FDRF = false;
bool DTMEN = false;
bool psBits[2]= {false,false}; //ps-0,1
bool gpioBits_b[4] = {false,false,false,false}; // Gpio 6,7,8,9
bool dccBits_b[7]= {false,false,false,false,false,false,false}; //Dcc 0,13,14,15,16

/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV32); // This will set the Linduino to have a 1MHz Clock
  LTC6813_init_cfg(TOTAL_IC, bms_ic);
  LTC6813_init_cfgb(TOTAL_IC,bms_ic);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6813_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,dccBits_a, dctoBits, UV, OV);
    LTC6813_set_cfgrb(current_ic,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,dccBits_b);   
  }   
  LTC6813_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6813_init_reg_limits(TOTAL_IC,bms_ic);
  print_menu();
}

/*!*********************************************************************
  \brief main loop
***********************************************************************/
void loop()
{
  if (Serial.available())           // Check for user input
  {
    uint32_t user_command;
    user_command = read_int();      // Read the user command
    Serial.println(user_command);
    run_command(user_command);
  }
}

/*!*****************************************
  \brief executes the user command
*******************************************/
void run_command(uint32_t cmd)
{
  int8_t error = 0;
  uint32_t conv_time = 0;
  uint32_t user_command;
  int8_t readIC=0;
  char input = 0;
  uint32_t adcstate =0;
  switch (cmd)
  {
  
    case 1: // Write and read Configuration Register
      wakeup_sleep(TOTAL_IC);
      LTC6813_wrcfg(TOTAL_IC,bms_ic);
      LTC6813_wrcfgb(TOTAL_IC,bms_ic);
      print_config();
      error = LTC6813_rdcfg(TOTAL_IC,bms_ic);
      check_error(error);
      error = LTC6813_rdcfgb(TOTAL_IC,bms_ic);
      check_error(error);
      print_rxconfig();
      break;
      
    case 2: // Start Cell ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
      conv_time = LTC6813_pollAdc();
      Serial.print(F("Cell conversion completed in:"));
      Serial.print(((float)conv_time/1000), 1);
      Serial.println(F("mS"));
      Serial.println();
      break;
    
    case 3: // Read Cell Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_rdcv(NO_OF_REG,TOTAL_IC,bms_ic); 
      check_error(error);
      print_cells(DATALOG_DISABLED);
      break;
    
    case 4: // Start GPIO ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6813_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
      conv_time = LTC6813_pollAdc();
      Serial.print(F("Auxiliary conversion completed in :"));
      Serial.print(((float)conv_time/1000), 1);
      Serial.println(F("mS"));
      Serial.println();
      break;
    
    case 5: // Read AUX Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_rdaux(NO_OF_REG,TOTAL_IC,bms_ic); 
      check_error(error);
      print_aux(DATALOG_DISABLED);
      break;
    
    case 6: // Start Status ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6813_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
      conv_time = LTC6813_pollAdc();
      Serial.print(F("Stat conversion completed in :"));
      Serial.print(((float)conv_time/1000), 1);
      Serial.println(F("mS"));
      Serial.println();
      break;
    
    case 7: // Read Status registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_rdstat(NO_OF_REG,TOTAL_IC,bms_ic);
      check_error(error);
      print_stat();
      break;
    
    case 8: // Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
      wakeup_sleep(TOTAL_IC);
      LTC6813_adcvax(ADC_CONVERSION_MODE,ADC_DCP);
      conv_time = LTC6813_pollAdc();
      Serial.println(F("Start Combined Cell Voltage and GPIO1, GPIO2 conversion completed"));
      Serial.print(F("conversion completed in:"));
      Serial.print(((float)conv_time/1000), 1);
      Serial.println(F("mS"));
      Serial.println();
      error = LTC6813_rdcv(NO_OF_REG, TOTAL_IC,bms_ic); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      Serial.println();
      error = LTC6813_rdaux(NO_OF_REG,TOTAL_IC,bms_ic); // Set to read back all aux registers
      check_error(error);
      print_aux1(DATALOG_DISABLED);
      Serial.println();
      break;
      
    case 9 : //Start Combined Cell Voltage and Sum of cells
      wakeup_sleep(TOTAL_IC);
      LTC6813_adcvsc(ADC_CONVERSION_MODE,ADC_DCP);
      conv_time = LTC6813_pollAdc();
      Serial.print(F("Combined Cell Voltage conversion completed in:"));
      Serial.print(((float)conv_time/1000), 1);
      Serial.println(F("mS"));
      Serial.println();
      error = LTC6813_rdcv(NO_OF_REG, TOTAL_IC,bms_ic); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      Serial.println();
      error = LTC6813_rdstat(NO_OF_REG,TOTAL_IC,bms_ic); // Set to read back all aux registers
      check_error(error);
      print_statsoc();
      Serial.println();
      break;
      
    case 10:  // Loop Measurements with out datalog output
      Serial.println(F("transmit 'm' to quit"));
      wakeup_sleep(TOTAL_IC);
      LTC6813_wrcfg(TOTAL_IC,bms_ic);
      LTC6813_wrcfgb(TOTAL_IC,bms_ic);
      while (input != 'm')
      {
        if (Serial.available() > 0)
        {
          input = read_char();
        }
        measurement_loop(DATALOG_DISABLED);
      
        delay(MEASUREMENT_LOOP_TIME);
      }
      print_menu();
      break;
      
    case 11: //Datalog print option Loop Measurements
      Serial.println(F("transmit 'm' to quit"));
      wakeup_sleep(TOTAL_IC);
      LTC6813_wrcfg(TOTAL_IC,bms_ic);
      while (input != 'm')
      {
        if (Serial.available() > 0)
        {
          input = read_char();
        }
        measurement_loop(DATALOG_ENABLED);
        delay(MEASUREMENT_LOOP_TIME);
       }
      Serial.println();
      print_menu();
      break;
      
    case 12: // Run the Mux Decoder Self Test
      wakeup_sleep(TOTAL_IC);
      LTC6813_diagn();
      LTC6813_pollAdc();
      error = LTC6813_rdstat(NO_OF_REG,TOTAL_IC,bms_ic); // Set to read back all aux registers
      check_error(error);
      error = 0;
      for (int ic = 0; ic<TOTAL_IC; ic++)
      {
        if (bms_ic[ic].stat.mux_fail[0] != 0) error++;
      }
      if (error==0) Serial.println(F("Mux Test: PASS "));
      else Serial.println(F("Mux Test: FAIL "));
      break;
      
    case 13:  // Run the ADC/Memory Self Test
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_run_cell_adc_st(CELL,TOTAL_IC,bms_ic,ADC_CONVERSION_MODE,ADCOPT);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and CELL Memory \n"));
      
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_run_cell_adc_st(AUX,TOTAL_IC, bms_ic,ADC_CONVERSION_MODE,ADCOPT);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and AUX Memory \n"));
      
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_run_cell_adc_st(STAT,TOTAL_IC, bms_ic,ADC_CONVERSION_MODE,ADCOPT);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and STAT Memory \n"));
      print_menu();
      break;

   case 14: // Run ADC Overlap self test
      wakeup_sleep(TOTAL_IC);
      error = (int8_t)LTC6813_run_adc_overlap(TOTAL_IC,bms_ic);
      if (error==0) Serial.println(F("Overlap Test: PASS "));
      else Serial.println(F("Overlap Test: FAIL"));
      Serial.println();
      break;
    
    case 15: // Run ADC Redundancy self test
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_run_adc_redundancy_st(ADC_CONVERSION_MODE,AUX,TOTAL_IC, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in AUX Measurement \n"));
      
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_run_adc_redundancy_st(ADC_CONVERSION_MODE,STAT,TOTAL_IC, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in STAT Measurement \n"));
      break;
      
    case 16: // Run open wire self test
      wakeup_sleep(TOTAL_IC);
      LTC6813_run_openwire(TOTAL_IC, bms_ic);
      break;
      
    case 17: //print pec counter
      print_pec();
      Serial.println();
      break;
    
    case 18:// Reset pec counter
      LTC6813_reset_crc_count(TOTAL_IC,bms_ic);
      Serial.println("PEC Counters is Reset");
      Serial.println();
      break;
       
    case 19: // Enable a discharge transistor
      Serial.print(F("Please enter the Spin number"));
      readIC = (int8_t)read_int();
      Serial.print(readIC);
      Serial.println();
      wakeup_sleep(TOTAL_IC);
      LTC6813_set_discharge(readIC,TOTAL_IC,bms_ic);
      LTC6813_wrcfg(TOTAL_IC,bms_ic);
      LTC6813_wrcfgb(TOTAL_IC,bms_ic);
      print_config();
      error = LTC6813_rdcfg(TOTAL_IC,bms_ic);
      check_error(error);
      error = LTC6813_rdcfgb(TOTAL_IC,bms_ic);
      check_error(error);
      print_rxconfig();
      break;
    
    case 20: // Clear all discharge transistors
      wakeup_sleep(TOTAL_IC);
      LTC6813_clear_discharge(TOTAL_IC,bms_ic);
      LTC6813_wrcfg(TOTAL_IC,bms_ic);
      LTC6813_wrcfgb(TOTAL_IC,bms_ic);
      print_config();
      error = LTC6813_rdcfg(TOTAL_IC,bms_ic);
      check_error(error);
      error = LTC6813_rdcfgb(TOTAL_IC,bms_ic);
      check_error(error);
      print_rxconfig();
      break;
      
   case 21://Write read pwm configation
      wakeup_sleep(TOTAL_IC);            
      // pwm configation data
      bms_ic[0].pwm.tx_data[0]= 0x55;
      bms_ic[0].pwm.tx_data[1]= 0x44;
      bms_ic[0].pwm.tx_data[2]= 0x33;
      bms_ic[0].pwm.tx_data[3]= 0x22;
      bms_ic[0].pwm.tx_data[4]= 0x11;
      bms_ic[0].pwm.tx_data[5]= 0x00;           
      LTC6813_wrpwm(TOTAL_IC,0,bms_ic);
      print_pwm(); 
         
      LTC6813_rdpwm(TOTAL_IC,0,bms_ic);       
      print_rxpwm();                              
      break;

   case 22: // Write S Control Register Group
      wakeup_sleep(TOTAL_IC);
      // S control data
      bms_ic[0].sctrl.tx_data[0]= 0x77;
      bms_ic[0].sctrl.tx_data[1]= 0x77;
      bms_ic[0].sctrl.tx_data[2]= 0x77;
      bms_ic[0].sctrl.tx_data[3]= 0x29;
      bms_ic[0].sctrl.tx_data[4]= 0x10;
      bms_ic[0].sctrl.tx_data[5]= 0x06;
      
      LTC6813_wrsctrl(TOTAL_IC,STREG,bms_ic);
      print_sctrl();  
      // start S Control pulsing
      LTC6813_stsctrl();
      LTC6813_pollAdc();
      Serial.println(F("Start S Control Pulsing"));
      Serial.println();
     // Read S Control Register Group
      error=LTC6813_rdsctrl(TOTAL_IC,STREG,bms_ic);
      check_error(error);
      print_rxsctrl();
      break;
      
    case 23:  //  Write PWM/S contol register B
      wakeup_sleep(TOTAL_IC);
      
      // pwm b and s control data
      bms_ic[0].pwmb.tx_data[0]= 0x14;
      bms_ic[0].pwmb.tx_data[1]= 0x74;
      bms_ic[0].pwmb.tx_data[2]= 0x38;
      bms_ic[0].sctrlb.tx_data[3]= 0x52;
      bms_ic[0].sctrlb.tx_data[4]= 0x10;
      bms_ic[0].sctrlb.tx_data[5]= 0x09;
      
      LTC6813_wrpsb(TOTAL_IC,bms_ic);
      print_pwm_sctrlb();
      
      LTC6813_rdpsb(TOTAL_IC,bms_ic);       
      print_rxpsb();   
      break;
        
   case 24: // Clear S Control Register Group
      wakeup_sleep(TOTAL_IC);
      LTC6813_clrsctrl();
      Serial.println(F("S Control Register Cleared"));
      error=LTC6813_rdsctrl(TOTAL_IC,STREG,bms_ic);
      check_error(error);
      print_rxsctrl();
      LTC6813_rdpsb(TOTAL_IC,bms_ic);       
      print_rxpsb();   
      break;
      
   case 25://SPI Communication 
      wakeup_sleep(TOTAL_IC);
      bms_ic[0].com.tx_data[0]= 0x80; //comm data to be transmitted
      bms_ic[0].com.tx_data[1]= 0x00;
      bms_ic[0].com.tx_data[2]= 0xA2;
      bms_ic[0].com.tx_data[3]= 0x20;
      bms_ic[0].com.tx_data[4]= 0xA3;
      bms_ic[0].com.tx_data[5]= 0x39;
      LTC6813_wrcomm(TOTAL_IC,bms_ic);               
      print_comm();
      LTC6813_stcomm();
      LTC6813_pollAdc();      
      Serial.println(F("SPI Communication completed"));
      Serial.println();
      error = LTC6813_rdcomm(TOTAL_IC,bms_ic);                       
      check_error(error);
      print_rxcomm();  
      break;
      
   case 26: // write byte I2C Communication on the GPIO Ports(using eeprom 24AA01)
      wakeup_sleep(TOTAL_IC);          
      bms_ic[0].com.tx_data[0]= 0x6A; // Icom(6)Start + i2c_address D0 (1010 0000)
      bms_ic[0].com.tx_data[1]= 0x00; // Fcom master ack  
      bms_ic[0].com.tx_data[2]= 0x00; // eeprom address D1 (0000 0000)
      bms_ic[0].com.tx_data[3]= 0x00; // Fcom master ack 
      bms_ic[0].com.tx_data[4]= 0x01; // Icom BLANCK D2 (0Xxx)
      bms_ic[0].com.tx_data[5]= 0x29; // Fcom master nack+stop        
      LTC6813_wrcomm(TOTAL_IC,bms_ic);// write comm register    
      print_comm();                   
      LTC6813_stcomm();
      LTC6813_pollAdc();
      Serial.println(F("i2c Communication completed"));
      Serial.println();              
      error = LTC6813_rdcomm(TOTAL_IC,bms_ic); // Read comm register                       
      check_error(error);
      print_rxcomm(); // Print comm register  
      break; 
      
   case 27: // Read byte data I2C Communication on the GPIO Ports(using eeprom 24AA01)
      wakeup_sleep(TOTAL_IC);         
      bms_ic[0].com.tx_data[0]= 0x6A; // Icom(6)Start + i2c_address D0 (1010 0000)
      bms_ic[0].com.tx_data[1]= 0x10; // Fcom master ack 
      bms_ic[0].com.tx_data[2]= 0x60; // again start i2c with i2c_address (1010 0001)
      bms_ic[0].com.tx_data[3]= 0x19; // fcom master nack + stop           
      LTC6813_wrcomm(TOTAL_IC,bms_ic);              
      LTC6813_stcomm();// i2c for write data in slave device    
      LTC6813_pollAdc();
      error = LTC6813_rdcomm(TOTAL_IC,bms_ic); // Read comm register                 
      check_error(error);
      print_rxcomm(); // Print comm register   
      break;  

    case 28: //  Enable MUTE
       wakeup_sleep(TOTAL_IC);
       LTC6813_wrcfg(TOTAL_IC,bms_ic);
       LTC6813_wrcfgb(TOTAL_IC,bms_ic);
       print_config();
       LTC6813_mute();
       
      Serial.println("Received Configuration register after enabling MUTE");
      error = LTC6813_rdcfg(TOTAL_IC,bms_ic);
      check_error(error);
      error = LTC6813_rdcfgb(TOTAL_IC,bms_ic);
      check_error(error);
      print_rxconfig();
      break;
      
    case 29: // TO enable UNMUTE 
      wakeup_sleep(TOTAL_IC);
      LTC6813_wrcfg(TOTAL_IC,bms_ic);
      LTC6813_wrcfgb(TOTAL_IC,bms_ic);
      print_config();
      LTC6813_unmute();
      
      Serial.println(" Received Configuration register after disabling MUTE");
      error = LTC6813_rdcfg(TOTAL_IC,bms_ic);
      check_error(error);
      error = LTC6813_rdcfgb(TOTAL_IC,bms_ic);
      check_error(error);
      print_rxconfig();
      break;
      
    case 30: // Clear all ADC measurement registers
      wakeup_sleep(TOTAL_IC);
      LTC6813_clrcell();
      LTC6813_clraux();
      LTC6813_clrstat();
      Serial.println(F("All Registers Cleared"));
      error = LTC6813_rdcv(NO_OF_REG, TOTAL_IC,bms_ic); // read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      error = LTC6813_rdaux(NO_OF_REG,TOTAL_IC,bms_ic); // read back all auxiliary registers
      check_error(error);
      print_aux(DATALOG_DISABLED);
      error = LTC6813_rdstat(NO_OF_REG,TOTAL_IC,bms_ic); // read back all status registers
      check_error(error);
      print_stat();         
      break; 
   
    case 'm': //prints menu
      print_menu();
      break;

    default:
      Serial.println(F("Incorrect Option"));
      break;
  }
}

void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  
  if (WRITE_CONFIG == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_wrcfg(TOTAL_IC,bms_ic);
    LTC6813_wrcfgb(TOTAL_IC,bms_ic);
    print_config();
  }

  if (READ_CONFIG == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC,bms_ic);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC,bms_ic); 
    check_error(error);
    print_rxconfig();
  }

  if (MEASURE_CELL == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
    LTC6813_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcv(0, TOTAL_IC,bms_ic);
    check_error(error);
    print_cells(datalog_en);

  }

  if (MEASURE_AUX == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
    LTC6813_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdaux(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
    check_error(error);
    print_aux(datalog_en);
  }

  if (MEASURE_STAT == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
    LTC6813_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdstat(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
    check_error(error);
    print_stat();
  }

  if(PRINT_PEC == ENABLED)
  {
    print_pec();
  }  
}
/*!****************************************************************************
  Function to check error flag and print PEC error message
 *****************************************************************************/
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/*!*********************************
  \brief Prints the main menu
***********************************/
void print_menu()
{
  Serial.println(F("Write and Read Configuration: 1                                | Loop measurements with datalog output : 11            | Write and Read of PWM : 21 "));
  Serial.println(F("Start Cell Voltage Conversion: 2                               | Run Mux Self Test : 12                                | Write and  Read of Scontrol : 22 "));
  Serial.println(F("Read Cell Voltages: 3                                          | Run ADC Self Test: 13                                 | Write and Read of PWM/S control Register B : 23 "));
  Serial.println(F("Start Aux Voltage Conversion: 4                                | ADC overlap Test : 14                                 | Clear S control register : 24 "));
  Serial.println(F("Read Aux Voltages: 5                                           | Run Digital Redundancy Test : 15                      | SPI Communication  : 25 "));
  Serial.println(F("Start Stat Voltage Conversion: 6                               | Open Wire Test : 16                                   | I2C Communication Write to Slave :26 "));
  Serial.println(F("Read Stat Voltages: 7                                          | Print PEC Counter: 17                                 | I2C Communication Read from Slave :27"));
  Serial.println(F("Start Combined Cell Voltage and GPIO1, GPIO2 Conversion: 8     | Reset PEC Counter: 18                                 | Enable MUTE : 28"));
  Serial.println(F("Start  Cell Voltage and Sum of cells : 9                       | Set Discharge: 19                                     | Disable MUTE : 29"));
  Serial.println(F("loop Measurements: 10                                          | Clear Discharge: 20                                   | Clear Registers: 30 "));
  Serial.println();
  Serial.println(F("Print 'm' for menu"));
  Serial.println(F("Please enter command: "));
  Serial.println();
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      Serial.print(", ");
      for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++)
      {

        Serial.print(" C");
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].cells.c_codes[i]*0.0001,4);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {
      Serial.print("Cells, ");
      for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++)
      {
        Serial.print(bms_ic[current_ic].cells.c_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints Open wire test results to the serial port
 *****************************************************************************/
void print_open()
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (bms_ic[current_ic].system_open_wire == 0)
    {
      Serial.print("No Opens Detected on IC: ");
      Serial.print(current_ic+1, DEC);
      Serial.println();
    }
    else
    {
      for (int cell=0; cell<bms_ic[0].ic_reg.cell_channels+1; cell++)
      {
        if ((bms_ic[current_ic].system_open_wire &(1<<cell))>0)
        {
          Serial.print(F("There is an open wire on IC: "));
          Serial.print(current_ic + 1,DEC);
          Serial.print(F(" Channel: "));
          Serial.println(cell,DEC);
        }
      }
    }
  }
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      for (int i=0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(bms_ic[current_ic].aux.a_codes[5]*0.0001,4);
      for (int i=6; i < 10; i++)
      {
        Serial.print(F(", GPIO-"));
        Serial.print(i,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        
      }
        Serial.println();
        Serial.print(" Flags : 0x");
        Serial.print(bms_ic[current_ic].aux.a_codes[11],HEX);
      Serial.println();
    }
    else
    {
      Serial.print("AUX, ");

      for (int i=0; i < 12; i++)
      {
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_stat()
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(" SOC:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[0]*0.0001*30,4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[1]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[2]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[3]*0.0001,4);
    Serial.println();
    Serial.print(F("Flags:"));
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.flags[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.flags[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.flags[2]);
     Serial.print(F("\tMux fail flag:"));
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.mux_fail[0]);
     Serial.print(F("\tTHSD:"));
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.thsd[0]);
    Serial.println();  
  }
  Serial.println();
}
/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6813
 to the serial port.
 ********************************************************************************/
void print_config()
{
  int cfg_pec;
  Serial.println(F("Written Configuration: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0;i<6;i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(bms_ic[current_ic].config.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6,&bms_ic[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
    
    Serial.print(F("CFGB IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0;i<6;i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(bms_ic[current_ic].configb.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6,&bms_ic[current_ic].configb.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
  }
  Serial.println();
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6813 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
  Serial.println(F("Received Configuration "));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0;i<6;i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(bms_ic[current_ic].config.rx_data[i]);
    };
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[7]);
    Serial.println();

    Serial.print(F("CFGB IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0;i<6;i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(bms_ic[current_ic].configb.rx_data[i]);
    };
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].configb.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].configb.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}

void print_pec()
{
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
      Serial.println("");
      Serial.print(bms_ic[current_ic].crc_count.pec_count,DEC);
      Serial.print(F(" : PEC Errors Detected on IC"));
      Serial.println(current_ic+1,DEC);
  }
}
/*!****************************************************************************
  \brief Prints GPIO voltage codes (GPIO1 & 2)
 *****************************************************************************/
void print_aux1(uint8_t datalog_en)
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      for (int i=0; i < 2; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
    else
    {
      Serial.print("AUX, ");

      for (int i=0; i < 12; i++)
      {
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}
/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port
 *****************************************************************************/
void print_statsoc()
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(" SOC:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[0]*0.0001*30,4);
    Serial.print(F(","));
  }

  Serial.println();
}
/*!****************************************************************************
  \brief Prints received data from COMM register onto the serial port
 *****************************************************************************/

void print_rxcomm()
{
  Serial.println(F("Received Data"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is written on COMM register onto the serial port
 *****************************************************************************/
void print_comm()
{
  int comm_pec;

  Serial.println(F("Written Data in COMM Resgiter: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    comm_pec = pec15_calc(6,&bms_ic[current_ic].com.tx_data[0]);
    serial_print_hex((uint8_t)(comm_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(comm_pec));
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is written on PWM register onto the serial port
 *****************************************************************************/
void print_pwm()
{
  int pwm_pec;

  Serial.println(F("Written Data in PWM: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    pwm_pec = pec15_calc(6,&bms_ic[current_ic].pwm.tx_data[0]);
    serial_print_hex((uint8_t)(pwm_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(pwm_pec));
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  \brief Prints received data from PWM register onto the serial port
 *****************************************************************************/
void print_rxpwm()
{
  Serial.println(F("Received Data in PWM"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is written on S Control register 
 *****************************************************************************/
void print_sctrl()
{
  int sctrl_pec;

  Serial.println(F("Written Data in s ctrl register: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    sctrl_pec = pec15_calc(6,&bms_ic[current_ic].sctrl.tx_data[0]);
    serial_print_hex((uint8_t)(sctrl_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(sctrl_pec));
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is read back from S Control register 
 *****************************************************************************/
void print_rxsctrl()
{
  Serial.println(F("Received Data in S control reg"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is written on PWM / S Control register 
 *****************************************************************************/
void print_pwm_sctrlb()
{
  int sctrlb_pec;

  Serial.println(F("Written Data in s ctrl register: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].pwmb.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwmb.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwmb.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    sctrlb_pec = pec15_calc(6,&bms_ic[current_ic].sctrlb.tx_data[0]);
    serial_print_hex((uint8_t)(sctrlb_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(sctrlb_pec));
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is read back from PWM/ S Control register B
 *****************************************************************************/
 void print_rxpsb()
 {
  Serial.println(F("Received Data in PWM/S control reg B :"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[7]);
    Serial.println();
  }
  Serial.println();
 }
