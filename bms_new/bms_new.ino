/************************* Includes ***************************/
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

#include "LTC6811.h"
#include "LTC681x.h"
#include "LT_SPI.h"
#include "Linduino.h"
#include "UserInterface.h"
// #include "LT_I2C.h"
// #include "QuikEval_EEPROM.h"

/************************* Defines *****************************/
/****** Stock ******/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
/****** Custom ******/
#define BMS_FAULT_PIN 2
/****** Custom ******/
#define stat_pin 2  // In response to the PCB design pinout

/**************** Local Function Declaration *******************/
/****** Stock ******/
void check_error(int error);
void print_cells(uint8_t datalog_en);
void print_conv_time(uint32_t conv_time);
/****** Custom ******/
void read_voltage();
void set_all_discharge();
void stop_all_discharge();
void balance();
void calculate();
void check_stat();
void reset_vmin();
void set_ic_discharge(   // Add to balance function formally when compeleted
    int Cell,            // The cell to be discharged
    uint8_t current_ic,  // The subsystem of the selected IC to be discharge
    uint8_t total_ic,    // Number of ICs in the system
    cell_asic *ic        // A two dimensional array that will store the data
);
void temp_detect();  // called by calculate

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
/****************** Stock *******************/
const uint8_t TOTAL_IC = 2;  //!< Number of ICs in the daisy chain

// ADC Command Configurations. See LTC681x.h for options.
const uint8_t ADC_OPT = ADC_OPT_DISABLED;          //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;  //!< ADC Mode
const uint8_t ADC_DCP = DCP_ENABLED;               //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT =
    CELL_CH_ALL;  //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT =
    AUX_CH_ALL;  //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT =
    STAT_CH_ALL;                      //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL;  //!< Register Selection
const uint8_t SEL_REG_A = REG_1;      //!< Register Selection
const uint8_t SEL_REG_B = REG_2;      //!< Register Selection

const uint16_t MEASUREMENT_LOOP_TIME = 500;  //!< Loop Time in milliseconds(ms)

// Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD =
    41000;  //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD =
    30000;  //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

// Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL
// CAPS
const uint8_t WRITE_CONFIG =
    DISABLED;  //!< This is to ENABLED or DISABLED writing into to configuration
               //!< registers in a continuous loop
const uint8_t READ_CONFIG =
    DISABLED;  //!< This is to ENABLED or DISABLED reading the configuration
               //!< registers in a continuous loop
const uint8_t MEASURE_CELL =
    ENABLED;  //!< This is to ENABLED or DISABLED measuring the cell voltages in
              //!< a continuous loop
const uint8_t MEASURE_AUX =
    DISABLED;  //!< This is to ENABLED or DISABLED reading the auxiliary
               //!< registers in a continuous loop
const uint8_t MEASURE_STAT =
    DISABLED;  //!< This is to ENABLED or DISABLED reading the status registers
               //!< in a continuous loop
const uint8_t PRINT_PEC =
    DISABLED;  //!< This is to ENABLED or DISABLED printing the PEC Error Count
               //!< in a continuous loop

cell_asic BMS_IC[TOTAL_IC];  //!< Global Battery Variable

/****************** Custom ******************/
double vmin[TOTAL_IC];
double vmax[TOTAL_IC];
double consvmin[TOTAL_IC];
unsigned long timer;
enum stats {
  fault,
  work,
  charge,
};

stats status;

/*********************************************************
 Set the configuration bits.
 Refer to the Configuration Register Group from data sheet.
**********************************************************/
bool REFON = true;    //!< Reference Powered Up Bit
bool ADCOPT = false;  //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false, false, true, true,
                      true};  //!< GPIO Pin Control // Gpio 1,2,3,4,5
uint16_t UV = UV_THRESHOLD;   //!< Under-voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD;   //!< Over-voltage Comparison Voltage
bool DCCBITS_A[12] = {
    false, false, false, false, false, false,
    false, false, false, false, false, false};  //!< Discharge cell switch //Dcc
                                                //!< 1,2,3,4,5,6,7,8,9,10,11,12
bool DCTOBITS[4] = {
    true, false, true,
    false};  //!< Discharge time value // Dcto 0,1,2,3 // Programed for 4 min
/*Ensure that Dcto bits are set according to the required discharge time. Refer
 * to the data sheet */

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

  timer = 0;
  calculate();
  reset_vmin();

  Serial.println("Vmin:");
  for (int i = 0; i < TOTAL_IC; i++) {
    Serial.print(consvmin[i]);
    Serial.print(", ");
  }
  Serial.print("\n");

  pinMode(BMS_FAULT_PIN, OUTPUT);

  pinMode(stat_pin, INPUT);
  if (digitalRead(stat_pin) == HIGH) {
    status = charge;
  } else if (digitalRead(stat_pin) == LOW) {
    status = work;
  }
  Serial.println("Setup completed");
}

void loop() {
  SPI.begin();
  quikeval_SPI_connect();
  spi_enable(
      SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock

  // check_stat();
  read_voltage();  // read and print the current voltage
  calculate();     // calculate minimal and maxium

  // Serial.print(status);
  // Serial.print(", ");

  if (millis() - timer >= 10000) {
    Serial.print("Balance\n");
    balance();
    timer = millis();
  }

  if (status == fault) {
    digitalWrite(BMS_FAULT_PIN, HIGH);
  } else {
    digitalWrite(BMS_FAULT_PIN, LOW);
  }
  delay(100);

  // testing area **************************
  if (Serial.read() == '5') {
    Serial.print(
        "***********************************balance****************************"
        "****\n");
    balance();
  } else if (Serial.read() == '6') {
    stop_all_discharge();
  }
}

/**************** Local Function Implementation ****************/
/****** Stock ******/
void check_error(int error) {
  if (error == -1) {
    // Serial.println(F("A PEC error was detected in the received data"));
  }
}

void print_cells(uint8_t datalog_en) {  // modified
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (datalog_en == 0) {
      // Serial.print(" IC ");
      // Serial.print(current_ic + 1, DEC);
      // Serial.print(": ");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        // Serial.print(" C");
        // Serial.print(i + 1, DEC);
        // Serial.print(":");
        // Set non-read cells to 0 rather than 6.5535 for the sake of
        // readability
        if (BMS_IC[current_ic].cells.c_codes[i] == 65535) {
          Serial.print(0, 4);
        } else {
          Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        }
        Serial.print(",");
      }
      // Serial.println();
    } else {
      // Serial.print(" Cells :");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        // Set non-read cells to 0 rather than 6.5535 for the sake of
        // readability
        if (BMS_IC[current_ic].cells.c_codes[i] == 65535) {
          Serial.print(0, 4);
        } else {
          Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        }
        Serial.print(",");
      }
    }
  }
  Serial.print("\n");
}

void print_conv_time(uint32_t conv_time) {
  uint16_t m_factor = 1000;  // to print in ms

  // Serial.print(F("Conversion completed in:"));
  // Serial.print(((float)conv_time / m_factor), 1);
  // Serial.println(F("ms \n"));
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

  // eliminate failed observation
  if (conv_time != 0) {
    print_cells(DATALOG_DISABLED);
  }
}

void set_all_discharge() {
  int8_t error = 0;
  uint32_t conv_time = 0;

  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
  check_error(error);  // Check error to enable the function

  for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
    wakeup_sleep(TOTAL_IC);
    LTC6811_set_discharge(i + 1, TOTAL_IC, BMS_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  }

  // Serial.println(F("----------start discharge----------"));
}

void stop_all_discharge() {
  int8_t error = 0;
  uint32_t conv_time = 0;

  // Serial.println(F("Stop all discharge !!"));
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
  check_error(error);

  // Serial.println(F("----------Stop balance----------"));

  wakeup_sleep(TOTAL_IC);
  LTC6811_clear_discharge(TOTAL_IC, BMS_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
}

void balance() {
  int8_t error = 0;
  uint32_t conv_time = 0;

  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
  check_error(error);  // Check error to enable the function

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      if (abs(BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
              consvmin[current_ic]) <= 0.1) {
        // build a customized one to determine the certain ic/cell to stop
        // discharge
        Serial.println("stop balance");
        LTC6811_clear_discharge(TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      } else if ((BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
                  consvmin[current_ic]) > 0.1) {
        Serial.println("start balance");
        set_ic_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      } else if ((BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
                  consvmin[current_ic]) < -0.1) {
        Serial.println("do nothing");
        // reset_vmin();
      }
    }
  }
}

void set_ic_discharge(
    int Cell,            // The cell to be discharged
    uint8_t current_ic,  // The subsystem of the selected IC to be discharge
    uint8_t total_ic,    // Number of ICs in the system
    cell_asic *ic        // A two dimensional array that will store the data
) {
  if ((Cell < 9) && (Cell != 0)) {
    ic[current_ic].config.tx_data[4] =
        ic[current_ic].config.tx_data[4] | (1 << (Cell - 1));
  } else if (Cell < 13) {
    ic[current_ic].config.tx_data[5] =
        ic[current_ic].config.tx_data[5] | (1 << (Cell - 9));
  }
}

void calculate() {  // calculate minimal and maxium
  int8_t error = 0;
  uint32_t conv_time = 0;

  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,
                       BMS_IC);  // Set to read back all cell voltage registers
  check_error(error);
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    vmin[current_ic] = 5;
    vmax[current_ic] = 0;
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      vmin[current_ic] > BMS_IC[current_ic].cells.c_codes[i] * 0.0001
          ? vmin[current_ic] = BMS_IC[current_ic].cells.c_codes[i] * 0.0001
          : 1;
      vmax[current_ic] < BMS_IC[current_ic].cells.c_codes[i] * 0.0001
          ? vmax[current_ic] = BMS_IC[current_ic].cells.c_codes[i] * 0.0001
          : 1;
    }
  }
  // temp_detect();
}

void check_stat() {
  switch (status) {
    case fault:
      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        if (vmax[current_ic] <= 4.25 && vmin[current_ic] >= 2.8) {
          status = work;
        }
      }
      if (status == fault) {
        stop_all_discharge();
        // exit(1);
      }
      break;
    case work:
      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        if (vmax[current_ic] >= 4.25 || vmin[current_ic] <= 2.8) {
          status = fault;
        } else if (vmax[current_ic] >= 4.2) {
          set_all_discharge();
        } else {
          stop_all_discharge();
        }
      }
      break;
    case charge:
      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        if (vmax[current_ic] >= 4.25 || vmin[current_ic] <= 2.8) {
          status = fault;
        } else if (vmax[current_ic] >= 4.12) {
          set_all_discharge();
        } else {
          stop_all_discharge();
        }
      }
      break;
    default:
      status = fault;
  }
}

void reset_vmin() {
  for (int i = 0; i < TOTAL_IC; i++) {
    consvmin[i] = vmin[i];  // Set up a costant Vminimum in case of the
                            // minumum become lower and lower
  }
}

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