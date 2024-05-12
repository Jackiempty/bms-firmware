/************************* Includes ***************************/
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

#include "LTC6811.h"
#include "LTC681x.h"
#include "LT_I2C.h"
#include "LT_SPI.h"
#include "Linduino.h"
#include "QuikEval_EEPROM.h"
#include "UserInterface.h"

/**************** Local Function Declaration *******************/
/****** Stock ******/
void check_error(int error);
/****** Custom ******/
void discharge();
void stop_discharge();
void calculate();
void starttowork();
void moniV(); // called by starttowork
void battery_charge_balance(); // called by starttowork

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
/****** Stock ******/
const uint8_t TOTAL_IC = 10;  //!< Number of ICs in the daisy chain

cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable

/****** Custom *****/
double vmin[TOTAL_IC];
double vmax[TOTAL_IC];
double consvmin[TOTAL_IC];

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

  if (shutdown_status) {
    digitalWrite(BMS_FAULT_PIN, HIGH);
  } else {
    digitalWrite(BMS_FAULT_PIN, LOW);
  }
  SPI.begin();
  quikeval_SPI_connect();
  spi_enable(
      SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
}

/**************** Local Function Implementation ****************/
/****** Stock ******/
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/****** Custom ******/
void discharge() {
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
  check_error(error);  // Check error to enable the function
  for (int i = 0; i < TOTAL_IC; i++) {
    consvmin[i] = vmin[i];  // Set up a costant Vminimum in case of the
                            // minumum become lower and lower
  }
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 > consvmin[current_ic]) {
        wakeup_sleep(TOTAL_IC);
        LTC6811_set_custom_discharge(i + 1, TOTAL_IC - (current_ic + 1),
                                     TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
        discharge_stat[current_ic][i] = true;
      }
    }
  }
  start_discharge = true;
  Serial.println(F("start discharge"));
  return;
}

void stop_discharge() {
  Serial.println(F("Stop all discharge !!"));
  wakeup_sleep(TOTAL_IC);
  RDS = 0;
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
  check_error(error);

  start_discharge = false;
  start_balance = false;
  Serial.println(F("Stop balance !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      wakeup_sleep(TOTAL_IC);
      LTC6811_clear_custom2_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
      LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    }
  }
  return;
}

void calculate() {  // calculate minimal and maxium (cmd 92)
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

  return;
}

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
} // called by starttowork

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
} // called by starttowork