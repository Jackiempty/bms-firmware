/************************* Includes ***************************/
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

#include "LTC6811.h"
#include "LTC681x.h"
// #include "LT_I2C.h"
#include "LT_SPI.h"
#include "Linduino.h"
// #include "QuikEval_EEPROM.h"
#include "UserInterface.h"

/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

/**************** Local Function Declaration *******************/
/****** Stock ******/
void check_error(int error);
void print_cells(uint8_t datalog_en);
void print_conv_time(uint32_t conv_time);
/****** Custom ******/
void read_voltage();
void discharge();
void stop_discharge();
void calculate();
// &&&&&& belows are waited to be eliminated &&&&&&&&
void starttowork();
void moniV();                   // called by starttowork
void battery_charge_balance();  // called by starttowork
void temp_detect();             // called by calculate

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
/****** Stock ******/
const uint8_t TOTAL_IC = 10;  //!< Number of ICs in the daisy chain

//ADC Command Configurations. See LTC681x.h for options.
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_ENABLED; //!< Discharge Permitted 
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection 
const uint8_t SEL_REG_A = REG_1; //!< Register Selection 
const uint8_t SEL_REG_B = REG_2; //!< Register Selection 

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = DISABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop

cell_asic BMS_IC[TOTAL_IC];  //!< Global Battery Variable

/****** Custom *****/
double vmin[TOTAL_IC];
double vmax[TOTAL_IC];
double consvmin[TOTAL_IC];

/*********************************************************
 Set the configuration bits. 
 Refer to the Configuration Register Group from data sheet. 
**********************************************************/
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false,false,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
uint16_t UV=UV_THRESHOLD; //!< Under-voltage Comparison Voltage
uint16_t OV=OV_THRESHOLD; //!< Over-voltage Comparison Voltage
bool DCCBITS_A[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCTOBITS[4] = {true, false, true, false}; //!< Discharge time value // Dcto 0,1,2,3 // Programed for 4 min 
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */

void setup() {
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(
      SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
  LTC6811_init_cfg(TOTAL_IC, BMS_IC);
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    LTC6811_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A,
                     DCTOBITS, UV, OV);
  }
  LTC6811_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);
}

void loop() {
  SPI.begin();
  quikeval_SPI_connect();
  spi_enable(
      SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
}

/**************** Local Function Implementation ****************/
/****** Stock ******/
void check_error(int error) {
  if (error == -1) {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

void print_cells(uint8_t datalog_en) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (datalog_en == 0) {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(": ");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print(" C");
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.println();
    } else {
      Serial.print(" Cells :");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

void print_conv_time(uint32_t conv_time) {
  uint16_t m_factor=1000;  // to print in ms

  Serial.print(F("Conversion completed in:"));
  Serial.print(((float)conv_time/m_factor), 1);
  Serial.println(F("ms \n"));
}
/****** Custom ******/
void read_voltage() {
  int8_t error = 0;
  uint32_t conv_time = 0;

  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  print_conv_time(conv_time);
  wakeup_idle(TOTAL_IC);
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
  check_error(error);
  print_cells(DATALOG_DISABLED);
}

void discharge() {
  int8_t error = 0;
  uint32_t conv_time = 0;

  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
  check_error(error);  // Check error to enable the function
  for (int i = 0; i < TOTAL_IC; i++) {
    // ******** need to change the usage of consvmin and vmin ********
    consvmin[i] = vmin[i];  // Set up a costant Vminimum in case of the
                            // minumum become lower and lower
  }

  for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
    wakeup_sleep(TOTAL_IC);
    LTC6811_set_discharge(i + 1, TOTAL_IC, BMS_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  }

  Serial.println(F("start discharge"));
}

void stop_discharge() {
  int8_t error = 0;
  uint32_t conv_time = 0;

  Serial.println(F("Stop all discharge !!"));
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
  check_error(error);

  Serial.println(F("Stop balance !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));

  // ******** make the set_discharge function back to stock one *********
  
  wakeup_sleep(TOTAL_IC);
  LTC6811_clear_discharge(TOTAL_IC, BMS_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  
}

void calculate() {  // calculate minimal and maxium (cmd 92)
  int8_t error = 0;
  uint32_t conv_time = 0;

  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
  check_error(error);
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
    }
  }
  temp_detect();
}

// ************ belows are waited to be eliminated **************
/*
void starttowork() {
  Serial.println("StartToWork");
  calculate();
  if (start_balance) {
    battery_charge_balance();
  }
  if (start_discharge) {
    moniV();
    Serial.println(F("Discharging!!!!!!!"));
  }
  return;
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

  return;
}  // called by starttowork

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

  return;
}  // called by starttowork

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

*/