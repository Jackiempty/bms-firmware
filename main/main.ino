/*! Analog Devices DC2350A-B Demonstration Board.
  LTC6811: Multicell Battery Monitors

  @verbatim
  NOTES
  Setup:8
    Set the terminal baud rate to 115200 and select the newline terminator.
    Ensure all jumpers on the demo board are installed in their default
positions from the factory. Refer to Demo Manual.

  USER INPUT DATA FORMAT:
  decimal : 1024
  hex     : 0x400
  octal   : 02000  (leading 0)
  binary  : B10000000000
  float   : 1024.0
  @endverbatim

  https://www.analog.com/en/products/LTC6811-1.html
  https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/dc2350a-b.html

********************************************************************************
  Copyright 2019(c) Analog Devices, Inc.

  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   - Neither the name of Analog Devices, Inc. nor the names of its
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.
   - The use of this software may or may not infringe the patent rights
     of one or more patent holders.  This license does not release you
     from the requirement that you obtain separate licenses from these
     patent holders to use this software.
   - Use of the software either in source or binary form, must be run
     on or directly connected to an Analog Devices Inc. component.

  THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*! @file
    @ingroup LTC6811-1*/
/************************************* Read me
******************************************* In this sketch book: -All Global
Variables are in Upper casing -All Local Variables are in lower casing -The
Function wakeup_sleep(TOTAL_IC) : is used to wake the LTC681x from sleep state.
   It is defined in LTC681x.cpp
  -The Function wakeup_idle(TOTAL_IC) : is used to wake the ICs connected in
daisy chain via the LTC6820 by initiating a dummy SPI communication. It is
defined in LTC681x.cpp
*******************************************************************************************/

/************************* Includes ***************************/
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Timer.h>
#include <due_can.h>
#include <math.h>
#include <mcp_can.h>
#include <stdint.h>

#include "LTC6811.h"
#include "LTC681x.h"
#include "LT_SPI.h"
#include "Linduino.h"
#include "UserInterface.h"
/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2
#define BMS_FAULT_PIN 2
#define BMS_FAULT_LENGTH 1024
/**************** Local Function Declaration *******************/
// void measurement_loop(uint8_t datalog_en);
void print_menu(void);
void print_wrconfig(void);
void print_wrconfigb(void);
void print_rxconfig(void);
void print_rxconfigb(void);
void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void print_stat(void);
void print_aux1(uint8_t datalog_en);
void print_sumofcells(void);
void check_mux_fail(void);
void print_selftest_errors(uint8_t adc_reg, int8_t error);
void print_overlap_results(int8_t error);
void print_digital_redundancy_errors(uint8_t adc_reg, int8_t error);
void print_open_wires(void);
void print_pec_error_count(void);
int8_t select_s_pin(void);
int8_t select_ic(void);
void print_wrpwm(void);
void print_rxpwm(void);
void print_wrsctrl(void);
void print_rxsctrl(void);
void print_wrpsb(uint8_t type);
void print_rxpsb(uint8_t type);
void print_wrcomm(void);
void print_rxcomm(void);
void check_mute_bit(void);
void print_conv_time(uint32_t conv_time);
void check_error(int error);
void serial_print_text(char data[]);
void serial_print_hex(uint8_t data);
char read_hex(void);
char get_char(void);
void maxerror(int CURIC, int err);
void minerror(int CURIC, int err);
void maxavererror(int CURIC, int err);
void minavererror(int CURIC, int err);
void differror(int CURIC, int err);

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
/*******************************************************************
********************************************************************
********************************************************************
********************************************************************
*******************************************************************/
const uint8_t TOTAL_IC = 8;  //!< Number of ICs in the daisy chain
/*******************************************************************
********************************************************************
********************************************************************
********************************************************************
*******************************************************************/
/********************************************************************
  ADC Command Configurations. See LTC681x.h for options
*********************************************************************/
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;  //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED;              //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT =
    CELL_CH_ALL;  //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT =
    AUX_CH_ALL;                       //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL;  //!< Register Selection
/*******************************************************
  Global Battery Variables received from 681x commands
  These variables store the results from the LTC6811
  register reads and the array lengths must be based
  on the number of ICs on the stack
 ******************************************************/
cell_asic BMS_IC[TOTAL_IC];  //!< Global Battery Variable
// cell_asic BMS_compare[TOTAL_IC]; // Copy the BMS_IC to make the sorting
/*************************************************************************
  Set configuration register. Refer to the data sheet
**************************************************************************/
bool REFON = true;    //!< Reference Powered Up Bit
bool ADCOPT = false;  //!< ADC Mode option bit
bool GPIOBITS_A[5] = {true, true, true, true,
                      true};  //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool GPIOBITS_B[4] = {true, true, true,
                      true};  //!< GPIO Pin Control // Gpio 6,7,8,9
bool DCCBITS_A[12] = {
    false, false, false, false, false, false, false,
    false, false, false, false, false};  //!< Discharge cell switch //Dcc
                                         //!< 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7] = {
    false, false, false, false,
    false, false, false};  //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = {
    true, false, true,
    false};  //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min
/*Ensure that Dcto bits are set according to the required discharge time. Refer
 * to the data sheet */
bool FDRF = false;                //!< Force Digital Redundancy Failure Bit
bool DTMEN = true;                //!< Enable Discharge Timer Monitor
bool PSBITS[2] = {false, false};  //!< Digital Redundancy Path Selection//ps-0,1

/*!**********************************************************************
  \brief  Initializes hardware and variables
  @return void
 ***********************************************************************/
Timer stat_print;
Timer update_vol;

/*Customize Global Variable*********************************************/
bool start_discharge = false;
bool start_balance = false;
bool shutdown_status = true;
bool discharge_stat[TOTAL_IC][12] = {false};  // 12 orginal 18 cell-num
//{"Max>3.73V","Min<2.5V","Max>avgvol","Min<avgvol","Max-Min>0.65V"};
/*Power charge*/
int relaypin[18] = {52, 50, 44, 42, 40, 38, 36, 34, 32,
                    30, 53, 51, 49, 47, 45, 43, 41, 39};  //[1~18]
int state[18] = {0};
int input = 0;
unsigned int RDS = 0;
/*Shutdown status*/
int maxfault[TOTAL_IC] = {0};
int minfault[TOTAL_IC] = {0};
int maxaverfault[TOTAL_IC] = {0};
int minaverfault[TOTAL_IC] = {0};
int difffault[TOTAL_IC] = {0};
int MXcount = 0;
int MIcount = 0;
int MXavercount = 0;
int MIavercount = 0;
int Diffcount = 0;
struct BMS_FAULTS_t {
  int FAULT_TYPE;
  int SEGMENT;
  int CELL_NUM;
  float VOLTAGE;
};
BMS_FAULTS_t BMS_FAULT_STATUS[BMS_FAULT_LENGTH];
short int BMS_FAULT_COUNTER = -1;
double vmin[TOTAL_IC];
double vmax[TOTAL_IC];
double diff[TOTAL_IC];
double consvmin[TOTAL_IC];
double avgvol[TOTAL_IC];
double sumvol[TOTAL_IC];
double totalvol;
unsigned long maxb[TOTAL_IC];
unsigned long minb[TOTAL_IC];
double current;
// GPIO pin 2 is disable
/******************current sensor******************/
/*double sum1 = 0;
  double sum2 = 0;
  int ct_count = 0;
  #define offset1 1893
  #define offset2 1900*/
/*//////can_bus_parameter/////////*/
int num = 0, count_ic = 0;
unsigned char msg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int id = 0;
unsigned char emergency_error[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char emergency_error2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
/*//////// SD card /////////*/
File myFile;
unsigned short u = 0;
/********************Setup()***************************/

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 18; i++) {
    pinMode(relaypin[i], OUTPUT);
    digitalWrite(relaypin[i], HIGH);
  }
  quikeval_SPI_connect();
  spi_enable(
      SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
  LTC6811_init_cfg(TOTAL_IC, BMS_IC);
  LTC6811_init_cfgb(TOTAL_IC, BMS_IC);

  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    LTC6811_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A,
                     DCTOBITS, 24000, 37300);  //!< Under voltage Over voltage
    LTC6811_set_cfgrb(current_ic, BMS_IC, FDRF, DTMEN, PSBITS, GPIOBITS_B,
                      DCCBITS_B);
  }
  shutdown_status = true;
  LTC6811_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);
  print_menu();

  for (int i = 0; i < 18; i++)  // Initialize the variable array
  {
    vmin[i] = 5;
    // consvmax[i] = 5;
  }
  starttowork();
  delay(100);
  Serial.println("1st starttowork completed");
  // temp_detect();
  voltage_print();
  Serial.println("1st voltage_print completed");
  stat_print.every(
      1500, starttowork1);  // Starttowork (loop routine/2s) : 1.calculate min
                            // and MAX of cell/ICs 2.Printout the Info. of
                            // Cell 3.check if start balance
  update_vol.every(
      1500, voltage_print);  // voltage_update(loop routine/500ms):1.Renew the
                             // min and MAX of cell/ICs 2.Send message to the
                             // CAN bus 3.check the error_avg.

  // clear all discharge before the Reset
  wakeup_sleep(TOTAL_IC);
  LTC6811_clear_discharge(TOTAL_IC, BMS_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  LTC6811_wrcfgb(TOTAL_IC, BMS_IC);

  /*while (CAN_OK != CAN.begin(CAN_500KBPS,1))              // init can bus :
    baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
    }
    Serial.println("CAN BUS Shield init ok!");*/

  pinMode(BMS_FAULT_PIN, OUTPUT);
  randomSeed(analogRead(A5));  // Random SOC
  SD.begin(4);
  // SD.remove("BMS_1.txt");
  Can0.begin(CAN_BPS_500K);
  Can0.watchFor();
}
/**********************************Start of Customize
 * Function*******************************/
void printFrame(CAN_FRAME &frame) {
  Serial.print("ID: 0x");
  Serial.print(frame.id, HEX);
  Serial.print(" Len: ");
  Serial.print(frame.length);
  Serial.print(" Data: ");
  for (int count = 0; count < frame.length; count++) {
    Serial.print(frame.data.bytes[count], HEX);
    Serial.print(" ");
  }
  Serial.print("\r\n");
}

void voltage_print() {
  Serial.println("voltage print waiting:50ms");
  Serial.println("Start Compare & sort");
  totalvol = 0;
  double hh = 0;
  for (int j = 0; j < TOTAL_IC; j++) {
    vmin[j] =
        5;  // vmin, vmax are recalculated in voltage_update & run_command(92)
            // each time in voltage_updated & run_command(92)
    vmax[j] = 0;
    avgvol[j] = 0;  // avgvol,sumvol are calculated in voltage_update
    sumvol[j] = 0;
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      Serial.print(BMS_IC[j].cells.c_codes[i] * 0.0001, 4);
      Serial.print(" ");
      sumvol[j] += BMS_IC[j].cells.c_codes[i] * 0.0001;
      if (j != 0) {
        hh += BMS_IC[j - 1].cells.c_codes[i] * 0.0001;
      }
      totalvol += BMS_IC[j].cells.c_codes[i] * 0.0001;
      if (vmin[j] > (float)(BMS_IC[j].cells.c_codes[i] * 0.0001)) {
        vmin[j] = BMS_IC[j].cells.c_codes[i] * 0.0001;
        minb[j] = i + 1;
      }
      if (vmax[j] < (float)(BMS_IC[j].cells.c_codes[i] * 0.0001)) {
        vmax[j] = BMS_IC[j].cells.c_codes[i] * 0.0001;
        maxb[j] = i + 1;
      }
    }
    Serial.print(" Min ");
    Serial.print(vmin[j], 4);
    Serial.print(" Max ");
    Serial.print(vmax[j], 4);
    // avgvol[j] = sumvol[j] / 18;
    avgvol[j] = (totalvol - hh) / 18.0;
    // totalvol+=sumvol[j];
    Serial.println("");
  }
  // current_detect();
  Maximum();
  Minimum();
  ///////////////error_average();
  ///////////////error_difference();
  error_temp();
  // error_current();
  // error_soc();
  emergency();
  /*******************SDcard***************************/
  /*u+=1;   //Shut down the printout
    if(u=5){
    SD_save5();
    u=0;
    }
    SD_save1();*/

  // SD_print(4);
  /*******************Canbus*************************/
  CAN_FRAME incoming;
  static unsigned long lastTime = 0;
  if (Can0.available() > 0) {
    Can0.read(incoming);
    printFrame(incoming);
    unsigned long t = millis();
    unsigned long start = millis();
    while (start < t + 200) {
      start = millis();
    }
  }
  // incoming.id = 224;
  switch (incoming.id) {
    case 192: {
      // Serial.print("Go to ");
      break;
    }
    case 224:
      // Serial.println("ooooooooo");
      send_msg_important();
      break;
    case 217:
      // Serial.println("PI");
      if (incoming.data.bytes[0] == 0 && incoming.data.bytes[1] == 0) {
        send_msg_important();  // 216
      }
      if (incoming.data.bytes[0] == 1 || incoming.data.bytes[0] == 3 ||
          incoming.data.bytes[0] == 5) {
        id = incoming.data.bytes[0];
        send_msg_battery();
        id = incoming.data.bytes[1];
        send_msg_battery();
      }
      break;
    default:
      Serial.print("");
      break;
  }
}
void starttowork1() {
  // wakeup_sleep(TOTAL_IC);
  // LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  // Serial.println("StartToWork");
  run_command(92);
  // sendData();         ////////////////
  // Serial.println();
  // Serial.println();
  // Serial.print(F("Sum voltage "));
  // Serial.print(F(" : "));
  // Serial.print(totalvol, 4);
  // Serial.println(F(",   "));
  // Serial.print(F("Total Current:"));
  // Serial.print(current, 4);
  // Serial.println(F(",   "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("Sum voltage of IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    Serial.print(sumvol[current_ic], 4);
    Serial.println(F(",   "));
    Serial.print(F("Average voltage of IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    Serial.print(avgvol[current_ic], 4);
    Serial.println(F(",   "));
    Serial.print(F("MAX difference Voltage_"));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    Serial.print(vmax[current_ic] - vmin[current_ic], 4);
    Serial.println(F(",   "));
    Serial.print(F("Highest Voltage_"));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" -> "));
    Serial.print(maxb[current_ic]);
    Serial.print(F("   "));
    Serial.print(vmax[current_ic], 4);
    Serial.print(F(",   "));
    Serial.print(F("Lowest Voltage_"));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" -> "));
    Serial.print(minb[current_ic]);
    Serial.print(F("   "));
    Serial.print(vmin[current_ic], 4);
    Serial.println(F(",   "));
    Serial.print(F("SOC"));
    Serial.print(F(" : "));
    Serial.print(100);
    Serial.print(F("% "));
    Serial.print(F(",   "));
    Serial.print(F("Temperature -> "));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    for (int i = 0; i < 8; i++) {
      if (i != 5 && i != 6) {
        Serial.print(BMS_IC[current_ic].aux.a_codes[i]);
        Serial.print(F(",   "));
      }
    }
    Serial.println();
  }
  print_cells(DATALOG_DISABLED);
  if (start_balance) {
    battery_charge_balance();
  }
  if (start_discharge) {
    moniV();
    Serial.println(F("Discharging!!!!!!!"));
  }
}
void voltage_update()  // update voltage per 500ms
{
  Serial.println("voltage updat");
  int8_t error = 0;
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
  check_error(error);
  totalvol = 0;
  double hh = 0;
  for (int j = 0; j < TOTAL_IC; j++) {
    vmin[j] =
        5;  // vmin, vmax are recalculated in voltage_update & run_command(92)
            // each time in voltage_updated & run_command(92)
    vmax[j] = 0;
    avgvol[j] = 0;  // avgvol,sumvol are calculated in voltage_update
    sumvol[j] = 0;
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      Serial.print(BMS_IC[j].cells.c_codes[i] * 0.0001, 4);
      Serial.print(" ");
      sumvol[j] += BMS_IC[j].cells.c_codes[i] * 0.0001;
      if (j != 0) {
        hh += BMS_IC[j - 1].cells.c_codes[i] * 0.0001;
      }
      totalvol += BMS_IC[j].cells.c_codes[i] * 0.0001;
      // vmin[j] > BMS_IC[j].cells.c_codes[i] * 0.0001 ? minb[j] = i + 1 : 1;
      // vmin[j] > BMS_IC[j].cells.c_codes[i] * 0.0001 ? vmin[j] =
      // BMS_IC[j].cells.c_codes[i] * 0.0001 : 1; vmax[j] <
      // BMS_IC[j].cells.c_codes[i] * 0.0001 ? maxb[j] = i + 1 : 1; vmax[j] <
      // BMS_IC[j].cells.c_codes[i] * 0.0001 ? vmax[j] =
      // BMS_IC[j].cells.c_codes[i] * 0.0001 : 1;
      /*compare min & max voltage in each segment(IC)*/
      if (vmin[j] > (float)(BMS_IC[j].cells.c_codes[i] * 0.0001)) {
        vmin[j] = BMS_IC[j].cells.c_codes[i] * 0.0001;
        minb[j] = i + 1;
      }
      if (vmax[j] < (float)(BMS_IC[j].cells.c_codes[i] * 0.0001)) {
        vmax[j] = BMS_IC[j].cells.c_codes[i] * 0.0001;
        maxb[j] = i + 1;
      }
    }
    Serial.print(" Min ");
    Serial.print(vmin[j], 4);
    Serial.print(" Max ");
    Serial.print(vmax[j], 4);
    // avgvol[j] = sumvol[j] / 18;
    avgvol[j] = (totalvol - hh) / 18.0;
    // totalvol+=sumvol[j];
    Serial.println("");
  }
  // current_detect();
  temp_detect();
  // Maximum();
  // Minimum();
  // error_average();
  // error_difference();
  error_temp();
  // error_current();
  // error_soc();
  emergency();
  /*******************SDcard***************************/
  /*u+=1;   //Shut down the printout
    if(u=5){
    SD_save5();
    u=0;
    }
    SD_save1();*/

  // SD_print(4);
  /*******************Canbus*************************/
  CAN_FRAME incoming;
  static unsigned long lastTime = 0;
  if (Can0.available() > 0) {
    Can0.read(incoming);
    printFrame(incoming);
    unsigned long t = millis();
    unsigned long start = millis();
    while (start < t + 200) {
      start = millis();
    }
  }
  // incoming.id = 224;
  switch (incoming.id) {
    case 192: {
      // Serial.print("Go to ");
      break;
    }
    case 224:
      // Serial.println("ooooooooo");
      send_msg_important();
      break;
    case 217:
      // Serial.println("PI");
      if (incoming.data.bytes[0] == 0 && incoming.data.bytes[1] == 0) {
        send_msg_important();  // 216
      }
      if (incoming.data.bytes[0] == 1 || incoming.data.bytes[0] == 3 ||
          incoming.data.bytes[0] == 5) {
        id = incoming.data.bytes[0];
        send_msg_battery();
        id = incoming.data.bytes[1];
        send_msg_battery();
      }
      break;
    default:
      Serial.print("");
      break;
  }
}

void starttowork() {
  // wakeup_sleep(TOTAL_IC);
  // LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  Serial.println("StartToWork");
  run_command(92);
  // sendData();         ////////////////
  Serial.println();
  Serial.println();
  Serial.print(F("Sum voltage "));
  Serial.print(F(" : "));
  Serial.print(totalvol, 4);
  Serial.println(F(",   "));
  Serial.print(F("Total Current:"));
  Serial.print(current, 4);
  Serial.println(F(",   "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("Sum voltage of IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    Serial.print(sumvol[current_ic], 4);
    Serial.println(F(",   "));
    Serial.print(F("Average voltage of IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    Serial.print(avgvol[current_ic], 4);
    Serial.println(F(",   "));
    Serial.print(F("MAX difference Voltage_"));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    Serial.print(vmax[current_ic] - vmin[current_ic], 4);
    Serial.println(F(",   "));
    Serial.print(F("Highest Voltage_"));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" -> "));
    Serial.print(maxb[current_ic]);
    Serial.print(F("   "));
    Serial.print(vmax[current_ic], 4);
    Serial.print(F(",   "));
    Serial.print(F("Lowest Voltage_"));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" -> "));
    Serial.print(minb[current_ic]);
    Serial.print(F("   "));
    Serial.print(vmin[current_ic], 4);
    Serial.println(F(",   "));
    Serial.print(F("SOC"));
    Serial.print(F(" : "));
    Serial.print(100);
    Serial.print(F("% "));
    Serial.print(F(",   "));
    Serial.print(F("Temperature -> "));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    for (int i = 0; i < 8; i++) {
      if (i != 2 && i != 5 && i != 6) {
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.01);
        Serial.print(F(",   "));
      }
    }
    Serial.println();
  }
  print_cells(DATALOG_DISABLED);
  if (start_balance) {
    battery_charge_balance();
  }
  if (start_discharge) {
    moniV();
    Serial.println(F("Discharging!!!!!!!"));
  }
}

void moniV()  // Every loop will Check out the Discharging cell is lower than
              // the lowest cell or not
{
  uint32_t conv_time = 0;
  int8_t error = 0;
  bool finish = true;
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
  check_error(error);            // Check error to enable the function

  Serial.println(F("Discharging Status :"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print("  ");

    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 < consvmin[current_ic]) {
        wakeup_sleep(TOTAL_IC);
        LTC6811_clear_custom2_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
        LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
        discharge_stat[current_ic][i] = false;  // Finish the Discharge
      }
      finish = finish &&
               !(discharge_stat[current_ic]
                               [i]);  // Have to compare the finish for all
                                      // finish == True that Discharged finish
      Serial.print("C");
      Serial.print(i);
      Serial.print(":");
      Serial.print(discharge_stat[current_ic][i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  if (finish) {
    start_discharge = false;
    Serial.println(
        F("!!!!!!!!!!!!!!!!!!!Discharge finished!!!!!!!!!!!!!!!!!!!!!"));
  }
}

void battery_charge_balance() {
  uint32_t conv_time = 0;
  int8_t error = 0;
  double chargeBias_buffer = 0;
  int light;
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
  check_error(error);            // Check error to enable the function

  run_command('AOV');
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    light = 0;
    chargeBias_buffer = 0;
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      if (light > 2) {
        chargeBias_buffer = 0.05;
      }
      if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 >
          BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30 / 18 + 0.05 +
              chargeBias_buffer) {
        light = light + 1;
        wakeup_sleep(TOTAL_IC);
        LTC6811_set_custom_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
        LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
      } else {
        wakeup_sleep(TOTAL_IC);
        LTC6811_clear_custom2_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
        LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
      }
    }
  }
}

/*void current_detect(){
  analogReadResolution(12);
  ct_count++;
  //Serial.println(analogRead(A0));
  sum1 += analogRead(A8);
  sum2 += analogRead(A9);

  if(ct_count==2){
    sum1 = (sum1/ct_count-offset1)*30/(3405-offset1);
    sum2 = (sum2/ct_count-offset2)*350/(3400-offset2);
    if(sum1 < 30 && sum1 > -30){
      current = sum1;
    }
    else{
      current = sum2;
    }
    Serial.print("Current:  ");
    Serial.println(current);
    Serial.print("Sum1:  ");
    Serial.println(sum1);
    Serial.print("Sum2:  ");
    Serial.println(sum2);
    ct_count=0;
    sum1=0;
    sum2=0;
  }
  }*/

void temp_detect() {
  uint32_t conv_time = 0;
  int8_t error = 0;
  wakeup_sleep(TOTAL_IC);
  LTC6811_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error =
      LTC6811_rdaux(0, TOTAL_IC, BMS_IC);  // Set to read back all aux registers
  check_error(error);
  delay(100);

  const float THSourceVoltage = 3.0;
  const int THRES = 10000;
  const float RT0 = 10000;  // 常溫 25度時的 NTC 電阻值
  const float RT1 = 32650;  // 0度時的 NTC 電阻值
  const float RT2 = 588.6;  // 105度時的 NTC 電阻值
  const float T0 = 298.15;  // 常溫 25度時的 Kelvin 值
  const float T1 = 273.15;  // 0度時的 Kelvin 值
  const float T2 = 378.15;  // 105度時的 Kelvin 值

  for (int current_ic = 0; current_ic < TOTAL_IC;
       current_ic++)  // 2 5 not work  //0 1 3 4 7 detect
  {
    for (int i = 0; i < 12; i++) {
      Serial.print(BMS_IC[current_ic].aux.a_codes[i]);
      Serial.print(", ");
    }

    Serial.println("###############");
    Serial.println();
  }
  float tf = 0;
  for (int current_ic = 0; current_ic < TOTAL_IC;
       current_ic++)  // 2 5 not work  //0 1 3 4 7 detect
  {
    for (int i = 0; i < 8;
         i++) {  // GPIO->V   10V/(5-V)=R=10*exp(3984*(1/T-1/298.15)) Rt = R
                 // *EXP(B*(1/T1-1/T2))
      if (BMS_IC[current_ic].aux.a_codes[i] != 0) {
        float value;
        float VoltageOut;
        float ROut;
        float beta;
        float Rx;
        float KelvinValue;

        beta = (log(RT1 / RT2)) / ((1 / T1) - (1 / T2));
        Rx = RT0 * exp(-beta / T0);

        VoltageOut = BMS_IC[current_ic].aux.a_codes[i] * 0.0001;

        ROut = THRES * VoltageOut /
               (THSourceVoltage - VoltageOut);  // 目前 NTC 電阻值
        KelvinValue = (beta / log(ROut / Rx));

        BMS_IC[current_ic].aux.a_codes[i] =
            KelvinValue - 273.15;  // Kelvin 轉 溫度C
      }
    }
  }
}

/*///////////Error Detecting///////////////
  /////////////////////////////////////////*/
/*Fcn Maximum check if any segment has cell's voltage >3.73V by comparing global
 * array vmin with 3.7, and vmax is calculated both in fcn voltage_update & fcn
 * starttowork(call another fcn run_command(92)) */
void Maximum() {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if ((float)vmax[current_ic] > 4.3) {
      Serial.print(F("IC(segment): "));
      Serial.print(current_ic);
      Serial.print(F("   vmax:  "));
      Serial.print(vmax[current_ic]);
      Serial.println(F(" is Over Voltage > 4.2V !"));
      maxerror(current_ic, 1);
      BMS_FAULT_STATUS[++BMS_FAULT_COUNTER] = {0, current_ic, -1,
                                               vmax[current_ic]};
    }

    else {
      maxerror(current_ic, 0);
    }
  }
}
/*Fcn Minimum check if any segment has cell's voltage <2.5V by comparing global
 * array vmin with 2.5, and vmin is calculated both in fcn voltage_update & fcn
 * starttowork(call another fcn run_command(92))*/
void Minimum() {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)

  {
    if (vmin[current_ic] < 2.5) {
      Serial.print(F("IC(segment): "));
      Serial.print(current_ic);
      Serial.print(F("   vmin:  "));
      Serial.print(vmin[current_ic]);
      Serial.println(F(" is Under Voltage < 2.5V !"));
      minerror(current_ic, 1);
      BMS_FAULT_STATUS[++BMS_FAULT_COUNTER] = {1, current_ic, -1,
                                               vmin[current_ic]};
    } else {
      minerror(current_ic, 0);
    }
  }
}
/*Fcn error_average check if any segment's vmax>  cells' avg. voltage in the
 * same segment, by compare global array vmax with avgvol.Global array  vmax &
 * avgvol are calculated*/
void error_average() {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (vmax[current_ic] > avgvol[current_ic] + 0.2) {
      Serial.print(F("IC(segment): "));
      Serial.print(current_ic);
      Serial.print(F("   vmax:  "));
      Serial.print(vmax[current_ic]);
      Serial.println(F(" is Over Average Voltage !"));
      maxavererror(current_ic, 1);
      BMS_FAULT_STATUS[++BMS_FAULT_COUNTER] = {2, current_ic, -1,
                                               vmax[current_ic]};
    }
    if (vmin[current_ic] < avgvol[current_ic] - 0.2) {
      Serial.print(F("IC(segment): "));
      Serial.print(current_ic);
      Serial.print(F("   vmin:  "));
      Serial.print(vmin[current_ic]);
      Serial.println(F("Under Average Voltage !"));
      minavererror(current_ic, 1);
      BMS_FAULT_STATUS[++BMS_FAULT_COUNTER] = {3, current_ic, -1,
                                               vmax[current_ic]};
    } else {
      maxavererror(current_ic, 0);
      minavererror(current_ic, 0);
    }
  }
}
/*Fcn error_difference check if any segment's max voltage difference >0.65V, by
 * compare vmax-vmin of each segment with 0.65, global array vmin & vmax are
 * calculated in fcn voltage_update  & fcn starttowork(call another fcn
 * run_command(92))*/
void error_difference() {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (vmax[current_ic] - vmin[current_ic] > 0.5) {
      Serial.print(F("IC(segment): "));
      Serial.print(current_ic);
      Serial.print(F("   vmax:  "));
      Serial.print(vmax[current_ic]);
      Serial.println(F(" is Over maximum Voltage difference !"));
      differror(current_ic, 1);
      BMS_FAULT_STATUS[++BMS_FAULT_COUNTER] = {4, current_ic, -1,
                                               vmax[current_ic]};
    } else {
      differror(current_ic, 0);
    }
  }
}
void error_temp() {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < 8; i++) {
      if (i != 2 && i != 5 && i != 6) {
        if (BMS_IC[current_ic].aux.a_codes[i] > 60) {
          // Serial.println(F("Over maximum Temperature !"));
          // shutdown_status = false;
          emergency_error[5] = 1;
        }
        if (BMS_IC[current_ic].aux.a_codes[i] <= 0) {
          // Serial.println(F("Temprature plug has gone !"));
          // shutdown_status = false;
          emergency_error[6] = 1;
        }
      }
    }
  }
}

void error_current() {
  if (current > 200) {
    Serial.println(F("Over maximum Current !"));
    shutdown_status = false;
    emergency_error2[0] = 1;
  }
  if (current < -15)  // charge the battery
  {
    Serial.println(F("Over minimum Current !"));
    shutdown_status = false;
    emergency_error2[1] = 1;
  }
}

/*void error_soc(){
  if (BMS_IC[0].aux.a_codes[0]*0.0001 < 10)
  {
    Serial.println(F("SOC < 10% !"));
    shutdown_status = true;
    emergency_error2[2]=1;
  }
  }*/

/*/////////////can_bus/////////////////////
  /////////////////////////////////////////*/
void msg_assign(unsigned int a) {
  int low_total = a % 256;
  int high_total = a / 256;
  switch (num) {
    case 0:
      msg[0] = low_total;
      msg[1] = high_total;
      break;
    case 1:
      msg[2] = low_total;
      msg[3] = high_total;
      break;
    case 2:
      msg[4] = low_total;
      msg[5] = high_total;
      break;
    case 3:
      msg[6] = id;
      msg[7] = 0;
      break;
    default:
      break;
  }
}

void send_msg(int a, unsigned char message[]) {
  /*Serial.println(a);
    for(int i=0;i<8;i++){
    Serial.print(message[i]);
    Serial.print(" ");
    }
    Serial.println();*/
  CAN_FRAME outgoing_1;
  outgoing_1.id = a;
  outgoing_1.extended = false;
  outgoing_1.length = 8;
  outgoing_1.data.bytes[0] = message[0];
  outgoing_1.data.bytes[1] = message[1];
  outgoing_1.data.bytes[2] = message[2];
  outgoing_1.data.bytes[3] = message[3];
  outgoing_1.data.bytes[4] = message[4];
  outgoing_1.data.bytes[5] = message[5];
  outgoing_1.data.bytes[6] = message[6];
  outgoing_1.data.bytes[7] = message[7];
  // Serial.print(outgoing_1.id, HEX);
  // Serial.print('\t');
  // Serial.println("haha");
  Can0.sendFrame(outgoing_1);
}

void send_msg_battery() {
  int k = 0;
  count_ic = id - 1;
  for (int i = 0; i < 6; i++) {
    for (num = 0; num < 4; num++) {
      msg_assign(BMS_IC[count_ic].cells.c_codes[k]);
      if (num == 3) {
      } else {
        k += 1;
      }
    }
    /*t = millis();
      start = millis();
      while(start<t+100){
      start = millis();
      }*/
    send_msg(208 + i, msg);
  }
  //....................send_msg_diff
  /*t = millis();
    start = millis();
    while(start<t+100){
    start = millis();
    }*/
  diff[count_ic] = vmax[count_ic] - vmin[count_ic];
  num = 0;
  msg_assign(avgvol[count_ic] * 10000);
  num = 1;
  msg_assign(diff[count_ic] * 10000);
  msg[4] = 0;
  msg[5] = 0;
  msg[6] = id;
  msg[7] = 0;
  send_msg(214, msg);
  //....................send_msg_extreme
  /*t = millis();
    start = millis();
    while(start<t+100){
    start = millis();
    }*/
  num = 0;
  msg_assign(vmax[count_ic] * 10000);
  num = 1;
  msg_assign(vmin[count_ic] * 10000);
  msg[4] = 0;
  msg[5] = 0;
  msg[6] = id;
  msg[7] = 0;
  send_msg(215, msg);
  //...................send_msg_temperature
  num = 0;
  msg_assign(BMS_IC[count_ic].aux.a_codes[0]);
  num = 1;
  msg_assign(BMS_IC[count_ic].aux.a_codes[1]);
  num = 2;
  msg_assign(BMS_IC[count_ic].aux.a_codes[3]);
  msg[6] = id;
  msg[7] = 0;
  send_msg(220, msg);

  num = 0;
  msg_assign(BMS_IC[count_ic].aux.a_codes[4]);
  num = 1;
  msg_assign(BMS_IC[count_ic].aux.a_codes[7]);
  num = 2;
  msg[4] = 0;
  msg[5] = 0;
  msg[6] = id;
  msg[7] = 0;
  send_msg(221, msg);
}
void send_msg_important() {
  num = 0;  // current
  // msg_assign(current*100);
  msg[0] = 0;
  msg[1] = 0;
  num = 1;
  msg_assign(totalvol * 10);
  num = 2;  // soc
  msg[4] = 99;
  // msg_assign(random(0,100));
  msg[5] = 0;
  msg[6] = 0;
  msg[7] = 0;
  send_msg(216, msg);

  num = 0;  // sumvol
  msg_assign(sumvol[0] * 100);
  num = 1;
  msg_assign(sumvol[1] * 100);
  num = 2;
  msg_assign(sumvol[2] * 100);
  send_msg(222, msg);

  num = 0;
  msg_assign(sumvol[3] * 100);
  num = 1;
  msg_assign(sumvol[4] * 100);
  num = 2;
  msg_assign(sumvol[5] * 100);
  send_msg(223, msg);
}

void emergency() {
  for (int i = 0; i < 8; i++) {
    if (emergency_error[i] == 1) {
      send_msg(218, emergency_error);
      emergency_error[i] = 0;
    }
  }
  for (int i = 0; i < 8; i++) {
    if (emergency_error2[i] == 1) {
      send_msg(219, emergency_error2);
      emergency_error2[i] = 0;
    }
  }
}

/*/////////////SD card////////////////*/
void SD_save1() {
  SD.begin(4);
  // Serial.print("\nWaiting for SD card ready...");
  if (!SD.begin(4)) {
    Serial.println("SD card Fail!");
    return;
  }
  // Serial.println("Success!");
  myFile = SD.open("BMS_1.txt", FILE_WRITE);  // 開啟檔案，一次僅能開啟一個檔案
  if (myFile) {
    myFile.print(F("Total voltage "));
    myFile.print(F(" : "));
    myFile.print(totalvol, 4);
    myFile.println(F(",   "));
    /*myFile.print(F("Total current "));
      myFile.print(F(" : "));
      //myFile.print(current,4);
      myFile.println(F(",   "));*/
  } else {
    Serial.println("\n open file error ");  // 無法開啟時顯示錯誤
  }
  myFile.close();
}

void SD_save5() {
  SD.begin(4);
  // Serial.print("\nWaiting for SD card ready...");
  if (!SD.begin(4)) {
    Serial.println("SD card Fail!");
    return;
  }
  // Serial.println("Success!");
  myFile = SD.open("BMS_5.txt", FILE_WRITE);  // 開啟檔案，一次僅能開啟一個檔案
  if (myFile) {                               // 假使檔案開啟正常
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      myFile.print(F("Average voltage of IC "));
      myFile.print(current_ic + 1);
      myFile.print(F(" : "));
      myFile.print(avgvol[current_ic], 4);
      myFile.print(F(",   "));

      /*myFile.print(F("SOC"));
        myFile.print(F(" : "));
        myFile.print(100);
        myFile.print(F("% "));
        myFile.println(F(",   "));*/

      myFile.print(F("MAX difference Voltage_"));
      myFile.print(F("IC "));
      myFile.print(current_ic + 1);
      myFile.print(F(" : "));
      myFile.print(vmax[current_ic] - vmin[current_ic], 4);
      myFile.print(F(",   "));

      myFile.print(F("Highest Voltage:"));
      myFile.print(vmax[current_ic], 4);
      myFile.print(F(",   "));

      myFile.print(F("Lowest Voltage:"));
      myFile.print(vmin[current_ic], 4);
      myFile.print(F("   ;   "));
    }
    myFile.println();
  }
  myFile.close();

  myFile = SD.open("BMS_T.txt", FILE_WRITE);  // 開啟檔案，一次僅能開啟一個檔案
  if (myFile) {                               // 假使檔案開啟正常
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      myFile.print(F("Temperature:"));
      myFile.print(F("IC "));
      myFile.print(current_ic + 1, DEC);
      myFile.print(",");
      for (int i = 0; i < 8; i++) {
        if (i != 2 && i != 5 && i != 6) {
          myFile.print(BMS_IC[current_ic].aux.a_codes[i] * 0.01);
          myFile.print(F(",   "));
        }
      }
      myFile.print("   ;   ");
    }
    myFile.println();
    // Serial.println("Completed !!");
    myFile.close();  // 關閉檔案
  }

  myFile = SD.open("BMS_B.txt", FILE_WRITE);  // 開啟檔案，一次僅能開啟一個檔案
  if (myFile) {                               // 假使檔案開啟正常
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      myFile.print("IC ");
      myFile.print(current_ic + 1, DEC);
      myFile.print(",");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        myFile.print(" C");
        myFile.print(i + 1, DEC);
        myFile.print(":");
        myFile.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        myFile.print(",");
      }
      myFile.print("   ;   ");
    }
    myFile.println();
    // Serial.println("Completed !!");
    myFile.close();  // 關閉檔案
  } else {
    Serial.println("\n open file error ");  // 無法開啟時顯示錯誤
  }
}
void SD_print(int a) {
  SD.begin(4);
  if (!SD.begin(4)) {
    Serial.println("Fail!");
    return;
  }
  // Serial.println("Success!");
  switch (a) {
    case 1:
      myFile = SD.open("BMS_1.txt");
      if (myFile) {                   // 判斷檔案是否存在
        while (myFile.available()) {  // 若還有可讀取的資料
          Serial.write(myFile.read());
        }
        myFile.close();  // 關閉檔案
      } else {
        Serial.println("error opening BMS_1.txt");
      }
      break;
    case 2:
      myFile = SD.open("BMS_5.txt");
      if (myFile) {                   // 判斷檔案是否存在
        while (myFile.available()) {  // 若還有可讀取的資料
          Serial.write(myFile.read());
        }
        myFile.close();  // 關閉檔案
      } else {
        Serial.println("error opening BMS_2.txt");
      }
      break;
    case 3:
      myFile = SD.open("BMS_B.txt");
      if (myFile) {                   // 判斷檔案是否存在
        while (myFile.available()) {  // 若還有可讀取的資料
          Serial.write(myFile.read());
        }
        myFile.close();  // 關閉檔案
      } else {
        Serial.println("error opening BMS_B.txt");
      }
      break;
    case 4:
      myFile = SD.open("BMS_T.txt");
      if (myFile) {                   // 判斷檔案是否存在
        while (myFile.available()) {  // 若還有可讀取的資料
          Serial.write(myFile.read());
        }
        myFile.close();  // 關閉檔案
      } else {
        Serial.println("error opening BMS_T.txt");
      }
      break;
  }
}
/***************************************End of Customize
 * Function****************************/

/*!*********************************************************************
  \brief Main loop
   @return void
***********************************************************************/
void loop() {
  uint32_t user_command;
  if (Serial.available())  // Check for user input
  {
    user_command = read_int();  // Read the user command
    if (user_command == 'm') {
      print_menu();
    } else {
      Serial.println(user_command);
      run_command(user_command);
    }
  }
  if (RDS >= 1) {
    RDS += 1;
  }
  if (RDS == 5000000) {
    run_command(88);
    delay(1000);
    run_command(87);
    RDS = 1;
  }

  stat_print.update();
  update_vol.update();

  if (shutdown_status) {
    digitalWrite(BMS_FAULT_PIN, HIGH);
  } else {
    /////////////////// BUFF
    digitalWrite(BMS_FAULT_PIN, LOW);
    ///////////////////

    /*   Serial.println("BMS Fault");
       for(int i=0;i<BMS_FAULT_COUNTER;i++){
         Serial.print("Fault ");
         Serial.print(BMS_FAULT_STATUS[i].FAULT_TYPE);
         Serial.print(" SEGMENT ");
         Serial.print(BMS_FAULT_STATUS[i].SEGMENT);
         Serial.print(" CELL NUM ");
         Serial.print(BMS_FAULT_STATUS[i].CELL_NUM);
         Serial.print(" VOLTAGE ");
         Serial.println(BMS_FAULT_STATUS[i].VOLTAGE,4);
       }
       Serial.print("BMS error:");
       Serial.println(BMS_FAULT_COUNTER);
       Serial.print(shutdown_status);
       delay(1000);*/
  }
  SPI.begin();
  quikeval_SPI_connect();
  spi_enable(
      SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
}
int FAULT_TYPE;
int SEGMENT;
int CELL_NUM;
double VOLTAGE;
/*!*****************************************
  \brief executes the user command
    @return void
*******************************************/
void run_command(uint32_t cmd) {
  uint8_t streg = 0;
  int8_t error = 0;
  uint32_t conv_time = 0;
  int8_t s_pin_read = 0;
  int8_t s_ic = 0;

  switch (cmd) {
    case 1:  // Write and read Configuration Register
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);   // Write into Configuration Register
      LTC6811_wrcfgb(TOTAL_IC, BMS_IC);  // Write into Configuration Register B
      print_wrconfig();
      print_wrconfigb();

      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);  // Read Configuration Register
      check_error(error);
      error =
          LTC6811_rdcfgb(TOTAL_IC, BMS_IC);  // Read Configuration Register B
      check_error(error);
      print_rxconfig();
      print_rxconfigb();
      break;

    case 2:  // Read Configuration Register
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      error = LTC6811_rdcfgb(TOTAL_IC, BMS_IC);
      check_error(error);
      print_rxconfig();
      print_rxconfigb();
      break;

    case 3:  // Start Cell ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      print_conv_time(conv_time);
      break;

    case 4:  // Read Cell Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error =
          LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      break;

    case 5:  // Start GPIO ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      /*Serial.print(F("Auxiliary conversion completed in :"));
        Serial.print(((float)conv_time/1000), 1);
        Serial.println(F("ms"));
        Serial.println();*/
      break;

    case 6:  // Read AUX Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdaux(0, TOTAL_IC,
                            BMS_IC);  // Set to read back all aux registers
      check_error(error);
      print_aux(DATALOG_DISABLED);
      break;

    case 23:  // Enable a discharge transistor
      s_pin_read = select_s_pin();
      wakeup_sleep(TOTAL_IC);
      LTC6811_set_discharge(s_pin_read, TOTAL_IC, BMS_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
      print_wrconfig();
      print_wrconfigb();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      error = LTC6811_rdcfgb(TOTAL_IC, BMS_IC);
      check_error(error);
      print_rxconfig();
      print_rxconfigb();
      break;

    case 24:  // clear discharge of seleted cell of all ICs
      s_pin_read = select_s_pin();
      wakeup_sleep(TOTAL_IC);
      LTC6811_clear_custom_discharge(s_pin_read, TOTAL_IC, BMS_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
      // print_wrconfig();
      // print_wrconfigb();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      error = LTC6811_rdcfgb(TOTAL_IC, BMS_IC);
      check_error(error);
      // print_rxconfig();
      // print_rxconfigb();
      break;

    case 28:  // Read byte data I2C Communication on the GPIO Ports(using eeprom
              // 24AA01)
      wakeup_sleep(TOTAL_IC);
      /************************************************************
        Communication control bits and communication data bytes.
        Refer to the data sheet.
      *************************************************************/
      for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        BMS_IC[current_ic].com.tx_data[0] =
            0x6A;  // Icom(6)Start + I2C_address D0 (1010 0000)
        BMS_IC[current_ic].com.tx_data[1] = 0x10;  // Fcom master ack
        BMS_IC[current_ic].com.tx_data[2] =
            0x60;  // again start I2C with I2C_address (1010 0001)
        BMS_IC[current_ic].com.tx_data[3] = 0x19;  // fcom master nack + stop
      }
      LTC6811_wrcomm(TOTAL_IC, BMS_IC);
      wakeup_idle(TOTAL_IC);
      // LTC6811_stcomm();// I2C for write data in slave device
      LTC6811_pollAdc();
      error = LTC6811_rdcomm(TOTAL_IC, BMS_IC);  // Read comm register
      check_error(error);
      print_rxcomm();  // Print comm register
      break;

    case 'm':  // prints menu
      print_menu();
      break;

    case 'AOV':  // Start Combined Cell Voltage and Sum of cells
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcvsc(ADC_CONVERSION_MODE, ADC_DCP);
      conv_time = LTC6811_pollAdc();
      // print_conv_time(conv_time);
      wakeup_idle(TOTAL_IC);
      error =
          LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
      check_error(error);
      // print_cells(DATALOG_DISABLED);
      wakeup_idle(TOTAL_IC);
      check_error(error);
      // print_avgofvoltage();
      break;
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case 79:  // set discharge of cells of selected ICs  for ID4 only
      Serial.println(F("Run 79, Discharge a segment 4!!"));
      // s_ic = TOTAL_IC-select_ic();
      // ////1->0(6->5),2->1(5->4),3->2(4->3),4->3(3->2),5->4(2->1),6->5(1->0)
      wakeup_sleep(TOTAL_IC);
      RDS = 1;
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print("current_ic:4");
        Serial.print(" cell:");
        Serial.print(i);
        Serial.print("= ");
        Serial.println(BMS_IC[4].cells.c_codes[i] * 0.0001);
        LTC6811_set_custom_discharge(i + 1, 2, TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
        LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
        discharge_stat[4][i] = true;
      }
      break;

    case 80:  // To charge battery at high current with assign relay discharging
      wakeup_sleep(TOTAL_IC);
      if (TOTAL_IC == 1) {
        // input =Serial.parseInt();
        input = select_s_pin();
        if (input < 1 || input > 18) {
          ;
        } else {
          if (state[input - 1] == 0) {
            digitalWrite(relaypin[input - 1], LOW);
            Serial.print("Battery ");
            Serial.print(input);
            Serial.println(" start to discharge");
            state[input - 1] = 1;
          }
        }
      }
      break;
    case 81:  // To charge battery at high current with assign relay no
              // discharging
      wakeup_sleep(TOTAL_IC);
      if (TOTAL_IC == 1) {
        // input =Serial.parseInt();
        input = select_s_pin();
        if (input < 1 || input > 18) {
          ;
        } else {
          if (state[input - 1] == 1) {
            digitalWrite(relaypin[input - 1], HIGH);
            Serial.print("Battery ");
            Serial.print(input);
            Serial.println(" stop to discharge");
            state[input - 1] = 0;
          }
        }
      }
      break;

    case 82:  // To Charge battery at high current with all relay discharging
      wakeup_sleep(TOTAL_IC);
      if (TOTAL_IC == 1) {
        Serial.println(
            "Auto balance discharging  Start");  // Use 50mv larger than mean
                                                 // voltage as discharge
                                                 // threshold
        for (int i = 0; i < 18; i++) {
          digitalWrite(relaypin[i], LOW);
          state[i] = 1;
        }
      }
      break;

    case 83:  // To Charge battery at high current with relay discharging
      wakeup_sleep(TOTAL_IC);
      if (TOTAL_IC == 1) {
        float ave = 0;
        RDS = 1;
        Serial.println(
            "Auto balance discharging");  // Use 50mv larger than mean voltage
                                          // as discharge threshold
        for (int i = 0; i < 18; i++) {
          ave += BMS_IC[0].cells.c_codes[i];
        }
        ave = ave / 18;
        for (int i = 0; i < 18; i++) {
          if ((BMS_IC[0].cells.c_codes[i] - ave) >= 0.05) {
            digitalWrite(relaypin[i], LOW);
            state[i] = 1;
          } else {
            digitalWrite(relaypin[i], HIGH);
            state[i] = 0;
          }
        }
      }
      break;

    case 84:  // To Charge battery at high current with no relay discharging
      wakeup_sleep(TOTAL_IC);
      if (TOTAL_IC == 1) {
        RDS = 0;
        Serial.println(
            "Auto balance discharging  Stop");  // Use 50mv larger than mean
                                                // voltage as discharge
                                                // threshold
        for (int i = 0; i < 18; i++) {
          digitalWrite(relaypin[i], HIGH);
          state[i] = 0;
        }
      }
      break;

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /*
        case 85://To Stop selected relay discharging
          wakeup_sleep(TOTAL_IC);
          if(TOTAL_IC==1)
          {
            Serial.println("Select discharge pin:");
            s_pin_read = select_s_pin();
            digitalWrite(s_pin_read,HIGH);
            Serial.println("Cell->",s_pin_read,"  stop discharging");
          }
          break;*/

    case 86:  // start discharge to the lowest voltage of all cells and ICs
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      //      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to
      //      read back all cell voltage registers
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);  // Check error to enable the function
      for (int i = 0; i < TOTAL_IC; i++) {
        consvmin[i] = vmin[i];  // Set up a costant Vminimum in case of the
                                // minumum become lower and lower
      }
      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
          if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 >
              consvmin[current_ic]) {
            wakeup_sleep(TOTAL_IC);
            LTC6811_set_custom_discharge(i + 1, TOTAL_IC - (current_ic + 1),
                                         TOTAL_IC, BMS_IC);
            LTC6811_wrcfg(TOTAL_IC, BMS_IC);
            LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
            discharge_stat[current_ic][i] = true;
          }
        }
      }
      start_discharge = true;
      Serial.println(F("start discharge"));
      break;

    case 87:  // charge balance
      /* Serial.println(F("Ha Ha !!"));
       wakeup_sleep(TOTAL_IC);
       LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
       conv_time = LTC6811_pollAdc();
       error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
       check_error(error);

       start_balance = true;
       Serial.println(F("start balance
       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")); break*/
      Serial.println(F("Run 87, Set discharge voltage balance all segment!!"));
      wakeup_sleep(TOTAL_IC);
      RDS = 1;
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      error =
          LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
      check_error(error);        // Check error to enable the function
      for (int i = 0; i < TOTAL_IC; i++) {
        consvmin[i] = 4.12 +  // Set up a costant Vminimum in case of the
                              // minumum become lower and lower
                      Serial.print("IC:");
        Serial.print(i);
        Serial.print(" consvmin is ");
        Serial.println(consvmin[i]);
      }
      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
          if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 >
              3.33)  // balance recommand 0.2A
          {
            Serial.print("current_ic:");
            Serial.print(current_ic);
            Serial.print(" cell:");
            Serial.print(i);
            Serial.print(" BMS_IC[current_ic].cells.c_codes[i] * 0.0001= ");
            Serial.println(BMS_IC[current_ic].cells.c_codes[i] * 0.0001);

            wakeup_sleep(TOTAL_IC);
            LTC6811_set_custom_discharge(i + 1, TOTAL_IC - (current_ic + 1),
                                         TOTAL_IC, BMS_IC);
            LTC6811_wrcfg(TOTAL_IC, BMS_IC);
            LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
            discharge_stat[current_ic][i] = true;
          }
        }
      }
      start_discharge = true;
      Serial.println(F("start discharge"));
      break;
      ;

    case 88:
      Serial.println(F("Run 88, Stop all discharge !!"));
      wakeup_sleep(TOTAL_IC);
      RDS = 0;
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
      check_error(error);

      start_discharge = false;
      start_balance = false;
      Serial.println(
          F("Stop balance !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));

      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
          wakeup_sleep(TOTAL_IC);
          LTC6811_clear_custom2_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
          LTC6811_wrcfg(TOTAL_IC, BMS_IC);
          LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
        }
      }
      break;

    case 89:  //*set* discharge of cells of selected ICs
      Serial.println(F("Run 89, Discharge a segment!!"));
      s_ic =
          TOTAL_IC -
          select_ic();  ////1->0(6->5),2->1(5->4),3->2(4->3),4->3(3->2),5->4(2->1),6->5(1->0)
      wakeup_sleep(TOTAL_IC);
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print("current_ic:");
        Serial.print(s_ic);
        Serial.print(" cell:");
        Serial.print(i);
        Serial.print("= ");
        Serial.println(BMS_IC[s_ic].cells.c_codes[i] * 0.0001);
        LTC6811_set_custom_discharge(i + 1, TOTAL_IC - s_ic, TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
        LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
        discharge_stat[s_ic][i] = true;
      }
      break;

    case 90:  //*set* discharge of seleted cell of selected ICs
      Serial.println(F("Set Discharge !!"));
      s_ic =
          TOTAL_IC -
          select_ic();  ////1->0(6->5),2->1(5->4),3->2(4->3),4->3(3->2),5->4(2->1),6->5(1->0)
      s_pin_read = select_s_pin();
      wakeup_sleep(TOTAL_IC);
      LTC6811_set_custom_discharge(s_pin_read, s_ic, TOTAL_IC, BMS_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
      // print_wrconfig();
      // print_wrconfigb();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      error = LTC6811_rdcfgb(TOTAL_IC, BMS_IC);
      check_error(error);
      // print_rxconfig();
      // print_rxconfigb();
      break;

    case 91:  //*clear* discharge of seleted cell of selected ICs
      Serial.println("Clear Discharge !!");
      s_ic = TOTAL_IC - select_ic();
      s_pin_read = select_s_pin();
      wakeup_sleep(TOTAL_IC);
      LTC6811_clear_custom2_discharge(s_pin_read, s_ic, TOTAL_IC, BMS_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
      // print_wrconfig();
      // print_wrconfigb();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      error = LTC6811_rdcfgb(TOTAL_IC, BMS_IC);
      check_error(error);
      // print_rxconfig();
      // print_rxconfigb();
      break;

    case 92:  // calculate minimal and maxium
      Serial.println("Run CMD92");
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      // conv_time = LTC6811_pollAdc();
      // print_conv_time(conv_time);
      error =
          LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
      check_error(error);
      // print_cells(DATALOG_DISABLED);
      for (int j = 0; j < TOTAL_IC; j++) {
        vmin[j] = 5;
        vmax[j] = 0;
        for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
          vmin[j] > BMS_IC[j].cells.c_codes[i] * 0.0001
              ? vmin[j] = BMS_IC[j].cells.c_codes[i] * 0.0001
              : 1;
          vmax[j] < BMS_IC[j].cells.c_codes[i] * 0.0001
              ? vmax[j] = BMS_IC[j].cells.c_codes[i] * 0.0001
              : 1;
          //          if(vmin[j]>BMS_IC[j].cells.c_codes[i]*0.0001 ){
          //            BMS_IC[j]=BMS_IC[j].cells.c_codes[i]*0.0001;
          //
          //          }
          //          else{
          //            BMS_IC[j]=1;
          //          }
        }
        Serial.print(F("min : "));
        Serial.print(vmin[j], 4);
        Serial.print(F("  max : "));
        Serial.println(vmax[j], 4);
      }
      temp_detect();
      break;

    default:
      char str_error[] = "Incorrect Option \n";
      serial_print_text(str_error);
      break;
  }
}

/*!**********************************************************************************************************************************************
  \brief For writing/reading configuration data or measuring cell voltages or
reading aux register or reading status register in a continuous loop
  @return void
*************************************************************************************************************************************************/
/*void measurement_loop(uint8_t datalog_en)
  {
  int8_t error = 0;
  char input = 0;

  Serial.println(F("Transmit 'm' to quit"));

  while (input != 'm')
  {
    if (Serial.available() > 0)
    {
      input = read_char();
    }

    if (WRITE_CONFIG == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
      print_wrconfig();
      print_wrconfigb();
    }

    if (READ_CONFIG == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      error = LTC6811_rdcfgb(TOTAL_IC, BMS_IC);
      check_error(error);
      print_rxconfig();
      print_rxconfigb();
    }

    if (MEASURE_CELL == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(0, TOTAL_IC, BMS_IC);
      check_error(error);
      print_cells(datalog_en);
    }

    if (MEASURE_AUX == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6811_adax(ADC_CONVERSION_MODE, AUX_CH_ALL);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdaux(0, TOTAL_IC, BMS_IC); // Set to read back all aux
  registers check_error(error); print_aux(datalog_en);
    }

    if (MEASURE_STAT == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdstat(0, TOTAL_IC, BMS_IC); // Set to read back all aux
  registers check_error(error); print_stat();
    }

    if (PRINT_PEC == ENABLED)
    {
      print_pec_error_count();
    }

    delay(100);
  }
  }
*/
/*!*********************************
  \brief Prints the main menu
  @return void
***********************************/
void print_menu(void) {
  Serial.println(F("List of LTC6811 Command:"));
  Serial.println(
      F("Write and Read Configuration: 1                            |SPI "
        "Communication: 28"));
  Serial.println(
      F("Read Configuration: 2                                      |Start "
        "discharge to the lowest voltage: 86"));
  Serial.println(
      F("Start Cell Voltage Conversion: 3                           |Start "
        "discharge to the average voltage: 87"));
  Serial.println(
      F("Read Cell Voltages: 4                                      |Stop "
        "discharging: 88"));
  Serial.println(
      F("Start Aux Voltage Conversion: 5                            |Set "
        "discharge of seleted cell of selected ICs: 90"));
  Serial.println(
      F("Read Aux Voltages: 6                                       |Clear "
        "discharge of seleted cell of selected ICs: 91"));
  Serial.println(
      F("Set Discharge of seleted cell of all ICs: 23               |Calculate "
        "minimal and maxium: 92 "));
  Serial.println(F("Clear Discharge of seleted cell of all ICs: 24"));
  Serial.println(F("Print 'm' for menu"));
  Serial.println(F("Please enter command: \n "));
}

/*!******************************************************************************
  \brief Prints the configuration data that is going to be written to the
 LTC6811 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfig(void) {
  int cfg_pec;
  Serial.println(F("Written Configuration A Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &BMS_IC[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println("\n");
  }
}

/*!******************************************************************************
  \brief Prints the Configuration Register B data that is going to be written to
  the LTC6811 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfigb(void) {
  int cfg_pec;
  Serial.println(F("Written Configuration B Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("CFGB IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].configb.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &BMS_IC[current_ic].configb.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println("\n");
  }
}

/*!*****************************************************************
  \brief Prints the configuration data that was read back from the
  LTC6811 to the serial port.
  @return void
 *******************************************************************/
void print_rxconfig(void) {
  Serial.println(F("Received Configuration A Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
    Serial.println("\n");
  }
}

/*!*****************************************************************
  \brief Prints the Configuration Register B that was read back from
  the LTC6811 to the serial port.
  @return void
 *******************************************************************/
void print_rxconfigb(void) {
  Serial.println(F("Received Configuration B Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("CFGB IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].configb.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].configb.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].configb.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
  @return void
 *************************************************************/
void print_cells(uint8_t datalog_en) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (datalog_en == 0) {
      Serial.print("IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(",");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print(" C");
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.println();
    } else {
      Serial.print(F(" Cells, "));
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(F(","));
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_aux(uint8_t datalog_en) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (datalog_en == 0) {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      for (int i = 0; i < 5; i++) {
        Serial.print(F(" GPIO-"));
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }

      for (int i = 6; i < 10; i++) {
        Serial.print(F(" GPIO-"));
        Serial.print(i, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
      }

      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(BMS_IC[current_ic].aux.a_codes[5] * 0.0001, 4);
      Serial.println();

      Serial.print(" OV/UV Flags : 0x");
      Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[11], HEX);
      Serial.println();
    } else {
      Serial.print(" AUX, ");

      for (int i = 0; i < 12; i++) {
        Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_stat(void) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    double itmp;

    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30, 4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    itmp =
        (double)((BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0076)) -
                 276);  // Internal Die Temperature(°C) = ITMP • (100 µV
                        // / 7.6mV)°C - 276°C
    Serial.print(itmp, 4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[2] * 0.0001, 4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[3] * 0.0001, 4);
    Serial.println();
    Serial.print(F(" OV/UV Flags:"));
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[2]);
    Serial.print(F("\tMux fail flag:"));
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.mux_fail[0]);
    Serial.print(F("\tTHSD:"));
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.thsd[0]);
    Serial.println();
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes (GPIO 1 & 2)
  @return void
 *****************************************************************************/
void print_aux1(uint8_t datalog_en) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (datalog_en == 0) {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      for (int i = 0; i < 2; i++) {
        Serial.print(F(" GPIO-"));
        Serial.print(i + 1, DEC);
        Serial.print(F(":"));
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(F(","));
      }
    } else {
      Serial.print(F("AUX, "));

      for (int i = 0; i < 12; i++) {
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(F(","));
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port6
 *****************************************************************************/
void print_sumofcells(void) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30, 4);
    Serial.print(F(","));
  }
  Serial.println("\n");
}

void print_avgofvoltage(void) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" AOV:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30 / 18, 4);
    Serial.print(F(","));
  }
  Serial.println("\n");
}

/*!****************************************************************
  \brief Function to check the MUX fail bit in the Status Register
   @return void
*******************************************************************/
void check_mux_fail(void) {
  int8_t error = 0;
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    Serial.print(" IC ");
    Serial.println(ic + 1, DEC);
    if (BMS_IC[ic].stat.mux_fail[0] != 0) error++;

    if (error == 0)
      Serial.println(F("Mux Test: PASS \n"));
    else
      Serial.println(F("Mux Test: FAIL \n"));
  }
}

/*!************************************************************
  \brief Prints Errors Detected during self test
   @return void
*************************************************************/
void print_selftest_errors(uint8_t adc_reg, int8_t error) {
  if (adc_reg == 1) {
    Serial.println("Cell ");
  } else if (adc_reg == 2) {
    Serial.println("Aux ");
  } else if (adc_reg == 3) {
    Serial.println("Stat ");
  }
  Serial.print(error, DEC);
  Serial.println(F(" : errors detected in Digital Filter and Memory \n"));
}

/*!************************************************************
  \brief Prints the output of  the ADC overlap test
   @return void
*************************************************************/
void print_overlap_results(int8_t error) {
  if (error == 0)
    Serial.println(F("Overlap Test: PASS \n"));
  else
    Serial.println(F("Overlap Test: FAIL \n"));
}

/*!************************************************************
  \brief Prints Errors Detected during Digital Redundancy test
   @return void
*************************************************************/
void print_digital_redundancy_errors(uint8_t adc_reg, int8_t error) {
  if (adc_reg == 2) {
    Serial.println("Aux ");
  } else if (adc_reg == 3) {
    Serial.println("Stat ");
  }

  Serial.print(error, DEC);
  Serial.println(F(" : errors detected in Measurement \n"));
}

/*!****************************************************************************
  \brief Prints Open wire test results to the serial port
 *****************************************************************************/
void print_open_wires(void) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (BMS_IC[current_ic].system_open_wire == 65535) {
      Serial.print("No Opens Detected on IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.println();
    } else {
      Serial.print(F("There is an open wire on IC "));
      Serial.print(current_ic + 1, DEC);
      Serial.print(F(" Channel: "));
      Serial.println(BMS_IC[current_ic].system_open_wire);
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
   \brief Function to print the number of PEC Errors
   @return void
 *****************************************************************************/
void print_pec_error_count(void) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.println("");
    Serial.print(BMS_IC[current_ic].crc_count.pec_count, DEC);
    Serial.print(F(" : PEC Errors Detected on IC"));
    Serial.println(current_ic + 1, DEC);
  }
  Serial.println("\n");
}

/*!****************************************************
  \brief Function to select the S pin for discharge
  @return void
 ******************************************************/
int8_t select_s_pin(void) {
  int8_t read_s_pin = 0;

  Serial.print(F("Please enter the Spin number: "));
  read_s_pin = (int8_t)read_int();
  Serial.println(read_s_pin);
  return (read_s_pin);
}

/*!****************************************************
  \brief Function to select the IC for discharge
  @return void
 ******************************************************/
int8_t select_ic(void) {
  int8_t read_s_ic = 0;

  Serial.print(F("Please enter the ic number: "));
  read_s_ic = (int8_t)read_int();
  Serial.println(read_s_ic);
  return (read_s_ic);
}

/*!****************************************************************************
  \brief prints data which is written on PWM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrpwm(void) {
  int pwm_pec;

  Serial.println(F("Written PWM Configuration: "));
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwm.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    pwm_pec = pec15_calc(6, &BMS_IC[current_ic].pwm.tx_data[0]);
    serial_print_hex((uint8_t)(pwm_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(pwm_pec));
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints received data from PWM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxpwm(void) {
  Serial.println(F("Received pwm Configuration:"));
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwm.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].pwm.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].pwm.rx_data[7]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief prints data which is written on S Control register
  @return void
 *****************************************************************************/
void print_wrsctrl(void) {
  int sctrl_pec;

  Serial.println(F("Written Data in Sctrl register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC: "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" Sctrl register group:"));
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrl.tx_data[i]);
    }

    Serial.print(F(", Calculated PEC: 0x"));
    sctrl_pec = pec15_calc(6, &BMS_IC[current_ic].sctrl.tx_data[0]);
    serial_print_hex((uint8_t)(sctrl_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(sctrl_pec));
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief prints data which is read back from S Control register
  @return void
 *****************************************************************************/
void print_rxsctrl(void) {
  Serial.println(F("Received Data:"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[i]);
    }

    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[7]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints data which is written on PWM/S control register group B onto
  the serial port
   @return void
 *****************************************************************************/
void print_wrpsb(uint8_t type) {
  int psb_pec = 0;

  Serial.println(F(" PWM/S control register group B: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (type == 1) {
      Serial.print(F(" IC: "));
      Serial.println(current_ic + 1, DEC);
      Serial.print(F(" 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[0]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[1]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[2]);

      Serial.print(F(", Calculated PEC: 0x"));
      psb_pec = pec15_calc(6, &BMS_IC[current_ic].pwmb.tx_data[0]);
      serial_print_hex((uint8_t)(psb_pec >> 8));
      Serial.print(F(", 0x"));
      serial_print_hex((uint8_t)(psb_pec));
      Serial.println("\n");
    } else if (type == 2) {
      Serial.print(F(" IC: "));
      Serial.println(current_ic + 1, DEC);
      Serial.print(F(" 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[3]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[4]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[5]);

      Serial.print(F(", Calculated PEC: 0x"));
      psb_pec = pec15_calc(6, &BMS_IC[current_ic].sctrlb.tx_data[0]);
      serial_print_hex((uint8_t)(psb_pec >> 8));
      Serial.print(F(", 0x"));
      serial_print_hex((uint8_t)(psb_pec));
      Serial.println("\n");
    }
  }
}

/*!****************************************************************************
  \brief Prints received data from PWM/S control register group B
   onto the serial port
   @return void
 *****************************************************************************/
void print_rxpsb(uint8_t type) {
  Serial.println(F(" PWM/S control register group B:"));
  if (type == 1) {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      Serial.print(F(" IC: "));
      Serial.print(F(" 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[0]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[1]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[2]);

      Serial.print(F(", Received PEC: 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[6]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[7]);
      Serial.println("\n");
    }
  } else if (type == 2) {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      Serial.print(F(" IC: "));
      Serial.println(current_ic + 1, DEC);
      Serial.print(F(" 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[3]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[4]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[5]);

      Serial.print(F(", Received PEC: 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[6]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[7]);
      Serial.println("\n");
    }
  }
}

/*!****************************************************************************
  \brief prints data which is written on COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrcomm(void) {
  int comm_pec;

  Serial.println(F("Written Data in COMM Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC- "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    comm_pec = pec15_calc(6, &BMS_IC[current_ic].com.tx_data[0]);
    serial_print_hex((uint8_t)(comm_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(comm_pec));
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints received data from COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxcomm(void) {
  Serial.println(F("Received Data in COMM register:"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC- "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
    Serial.println("\n");
  }
}

/*!********************************************************************
  \brief Function to check the Mute bit in the Configuration Register
   @return void
**********************************************************************/
void check_mute_bit(void) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" Mute bit in Configuration Register B: 0x"));
    serial_print_hex((BMS_IC[current_ic].configb.rx_data[1]) & (0x80));
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Function to print the Conversion Time
  @return void
 *****************************************************************************/
void print_conv_time(uint32_t conv_time) {
  uint16_t m_factor = 1000;  // to print in ms

  Serial.print(F("Conversion completed in:"));
  Serial.print(((float)conv_time / m_factor), 1);
  Serial.println(F("ms \n"));
}

/*!****************************************************************************
  \brief Function to check error flag and print PEC error message
  @return void
 *****************************************************************************/
void check_error(int error) {
  if (error == -1) {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/*!************************************************************
  \brief Function to print text on serial monitor
  @return void
*************************************************************/
void serial_print_text(char data[]) { Serial.println(data); }

/*!****************************************************************************
   \brief Function to print in HEX form
   @return void
 *****************************************************************************/
void serial_print_hex(uint8_t data) {
  if (data < 16) {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  } else
    Serial.print((byte)data, HEX);
}

/*!*****************************************************************************
  \brief Hex conversion constants
 *******************************************************************************/
char hex_digits[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                       '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

/*!************************************************************
  \brief Global Variables
 *************************************************************/
char hex_to_byte_buffer[5] = {'0', 'x', '0', '0', '\0'};

/*!************************************************************
  \brief Buffer for ASCII hex to byte conversion
 *************************************************************/
char byte_to_hex_buffer[3] = {'\0', '\0', '\0'};

/*!*****************************************************************************
  \brief Read 2 hex characters from the serial buffer and convert them to a byte
  @return char data Read Data
 ******************************************************************************/
char read_hex(void) {
  byte data;
  hex_to_byte_buffer[2] = get_char();
  hex_to_byte_buffer[3] = get_char();
  get_char();
  get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return (data);
}

/*!************************************************************
  \brief Read a command from the serial port
  @return char
 *************************************************************/
char get_char(void) {
  // read a command from the serial port
  while (Serial.available() <= 0);
  return (Serial.read());
}

void maxerror(int CURIC, int err) {
  maxfault[CURIC] = maxfault[CURIC] + err;
  MXcount++;
  if (MXcount == TOTAL_IC) {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      if (maxfault[current_ic] > 3) {
        shutdown_status = false;
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Maxerror fault");
        emergency_error[0] = 1;
      } else {
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Maxerror fine");
        Serial.print("Max fault = ");
        Serial.println(maxfault[CURIC]);
      }
    }
    MXcount = 0;
  }
}

void minerror(int CURIC, int err) {
  minfault[CURIC] = minfault[CURIC] + err;
  MIcount++;
  if (MIcount == TOTAL_IC) {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      if (minfault[current_ic] > 3) {
        shutdown_status = false;
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Minerror fault");
        emergency_error[1] = 1;
      } else {
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Minerror fine");
        Serial.print("Min fault = ");
        Serial.println(minfault[CURIC]);
      }
    }
    MIcount = 0;
  }
}

void maxavererror(int CURIC, int err) {
  maxaverfault[CURIC] = maxaverfault[CURIC] + err;
  MXavercount++;
  if (MXavercount == TOTAL_IC) {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      if (maxaverfault[current_ic] > 3) {
        shutdown_status = false;
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Maxaver error fault");
        emergency_error[2] = 1;
      } else {
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Maxaver error fine");
        Serial.print("Maxaver fault = ");
        Serial.println(maxaverfault[CURIC]);
      }
    }
    MXavercount = 0;
  }
}
void minavererror(int CURIC, int err) {
  minaverfault[CURIC] = minaverfault[CURIC] + err;
  MIavercount++;
  if (MIavercount == TOTAL_IC) {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      if (minaverfault[current_ic] > 3) {
        shutdown_status = false;
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Minaver error fault");
        emergency_error[3] = 1;
      } else {
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Minaver error fine");
        Serial.print("Minaver fault = ");
        Serial.println(minaverfault[CURIC]);
      }
    }
    MIavercount = 0;
  }
}

void differror(int CURIC, int err) {
  difffault[CURIC] = difffault[CURIC] + err;
  Diffcount++;
  if (Diffcount == TOTAL_IC) {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
      if (difffault[current_ic] > 3) {
        shutdown_status = false;
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Diff error fault");
        emergency_error[4] = 1;
      } else {
        Serial.print("current ic=");
        Serial.print(current_ic);
        Serial.println(" Diff error fine");
        Serial.print("Diff fault = ");
        Serial.println(difffault[CURIC]);
      }
    }
    Diffcount = 0;
  }
}
