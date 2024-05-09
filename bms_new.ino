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

/****** Custom ******/
void discharge();
void stop_discharge();
void starttowork();
void calculate();

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
  // print_menu();
}

void loop() {
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
  }
  SPI.begin();
  quikeval_SPI_connect();
  spi_enable(
      SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
}

/**************** Local Function Implementation ****************/
void discharge() {
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
      if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 > consvmin[current_ic]) {
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
  return;
}

void stop_discharge() {
  Serial.println(F("Run 88, Stop all discharge !!"));
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
      LTC6811_wrcfgb(TOTAL_IC, BMS_IC);
    }
  }
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

void calculate() {  // calculate minimal and maxium (cmd 92)
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  // conv_time = LTC6811_pollAdc();
  // print_conv_time(conv_time);
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
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
      // if(vmin[j]>BMS_IC[j].cells.c_codes[i]*0.0001 ){
      //   BMS_IC[j]=BMS_IC[j].cells.c_codes[i]*0.0001;

      // }
      // else{
      //   BMS_IC[j]=1;
      // }
    }
    Serial.print(F("min : "));
    Serial.print(vmin[j], 4);
    Serial.print(F("  max : "));
    Serial.println(vmax[j], 4);
  }
  temp_detect();

  return;
}