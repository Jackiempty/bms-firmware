/************************* Includes ***************************/
#include <Arduino.h>
#include <DueTimer.h>
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
#define STATE_PIN 3  // In response to the PCB design pinout

/**************** Local Function Declaration *******************/
/****** Stock ******/
void check_error(int error);
void print_cells(uint8_t datalog_en);
void print_conv_time(uint32_t conv_time);
/****** Custom ******/
void Isr();
void work_loop();
void charge_loop();
void read_voltage();
void set_all_discharge();
void stop_all_discharge();
void balance(double min_thr, double max_thr);
void check_stat();
void calculate();
void reset_vmin();
void set_ic_discharge(   // Add to balance function formally when compeleted
    int Cell,            // The cell to be discharged
    uint8_t current_ic,  // The subsystem of the selected IC to be discharge
    cell_asic *ic        // A two dimensional array that will store the data
);
void stop_ic_discharge(
    int Cell,            // The cell to stop discharging
    uint8_t current_ic,  // The subsystem of the selected IC to discharging
    cell_asic *ic        // A two dimensional array that will store the data
);
void temp_detect();  // called by calculate
void error_temp();   // detect temperature rules violation
/****** Test ******/
void select(int ic, int cell);

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
/****************** Stock *******************/
const uint8_t TOTAL_IC = 2;  //!< Number of ICs in the daisy chain

// ADC Command Configurations. See LTC681x.h for options.
const uint8_t ADC_OPT = ADC_OPT_DISABLED;
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;
const uint8_t ADC_DCP = DCP_ENABLED;
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;
const uint8_t SEL_ALL_REG = REG_ALL;
const uint8_t SEL_REG_A = REG_1;
const uint8_t SEL_REG_B = REG_2;

const uint16_t MEASUREMENT_LOOP_TIME = 500;

// Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000;
const uint16_t UV_THRESHOLD = 30000;

const uint8_t WRITE_CONFIG = DISABLED;
const uint8_t READ_CONFIG = DISABLED;
const uint8_t MEASURE_CELL = ENABLED;
const uint8_t MEASURE_AUX = DISABLED;
const uint8_t MEASURE_STAT = DISABLED;
const uint8_t PRINT_PEC = DISABLED;

cell_asic BMS_IC[TOTAL_IC];  //!< Global Battery Variable

/****************** Custom ******************/
double vmin[TOTAL_IC];
double vmax[TOTAL_IC];
double consvmin[TOTAL_IC];
enum stats {
  FAULT,
  WORK,
  CHARGE,
};
stats status;

/*********************************************************
 Set the configuration bits.
 Refer to the Configuration Register Group from data sheet.
**********************************************************/
bool REFON = true;    //!< Reference Powered Up Bit
bool ADCOPT = false;  //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false, false, true, true, true};
uint16_t UV = UV_THRESHOLD;  //!< Under-voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD;  //!< Over-voltage Comparison Voltage
bool DCCBITS_A[12] = {false, false, false, false, false, false,
                      false, false, false, false, false, false};
bool DCTOBITS[4] = {true, false, true, false};

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

  // Not quite yet, hence commented
  // Timer0.attachInterrupt(Isr).setFrequency(10).start();
  Serial.println("Vmin:");
  calculate();
  reset_vmin();

  
  for (int i = 0; i < TOTAL_IC; i++) {
    Serial.print(consvmin[i]);
    Serial.print(", ");
  }
  Serial.print("\n");

  pinMode(BMS_FAULT_PIN, OUTPUT);

  pinMode(STATE_PIN, INPUT);
  (digitalRead(STATE_PIN) == HIGH) ? status = CHARGE : status = WORK;

  Serial.println(F("Setup completed"));
}

void loop() {
  // check_stat();
  read_voltage();  // read and print the current voltage
  calculate();     // calculate minimal and maxium
  temp_detect();
  
  delay(1000);
  // *********************** testing area **************************
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case '5':
        Serial.print("******* dicharge all *******\n");
        set_all_discharge();
        break;
      case '6':
        Serial.print("****** stop discharge ******\n");
        stop_all_discharge();
        break;
      case '7':
        Serial.print("********** select **********\n");
        select(0, 3);
        select(1, 8);
        break;
      case '8':
        Serial.print("********* reset vmin *******\n");
        reset_vmin();
        break;
      default:
        Serial.print("******** do nothing ********\n");
        break;
    }
  }
}

/**************** Local Function Implementation ****************/
/****** Stock ******/
void check_error(int error) {
  if (error == -1) {
    Serial.println(F("A PEC error was detected in the received data"));
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

        // Set non-read cells to 0 rather than 6.5535
        if (BMS_IC[current_ic].cells.c_codes[i] == 65535) {
          Serial.print(float(0), 4);
        } else {
          Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        }
        Serial.print(", ");
      }
      // Serial.println();
    } else {
      // Serial.print(" Cells :");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        // Set non-read cells to 0 rather than 6.5535
        if (BMS_IC[current_ic].cells.c_codes[i] == 65535) {
          Serial.print(float(0), 4);
        } else {
          Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        }
        Serial.print(", ");
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
void Isr() {  // Interrupt main
  check_stat();
}

void work_loop() {  // thresholds are yet to be determined
  Serial.print(F("Work\n"));
  reset_vmin();
  // balance(); // Arg aka thresholds are yet to be determined
}

void charge_loop() {  // thresholds are yet to be determined
  Serial.print(F("Charge\n"));
  reset_vmin();
  // balance(); // Arg aka thresholds are yet to be determined
}

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
  for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
    wakeup_sleep(TOTAL_IC);
    LTC6811_set_discharge(i + 1, TOTAL_IC, BMS_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  }
  wakeup_idle(TOTAL_IC);
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
  check_error(error);  // Check error to enable the function

  Serial.println(F("--------- start discharge ---------"));
}

void stop_all_discharge() {
  int8_t error = 0;
  uint32_t conv_time = 0;

  wakeup_sleep(TOTAL_IC);
  LTC6811_clear_discharge(TOTAL_IC, BMS_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  wakeup_idle(TOTAL_IC);
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
  check_error(error);

  Serial.println(F("---------- stop discharge ----------"));
}

void check_stat() {
  stop_all_discharge();
  read_voltage();  // read and print the current voltage
  calculate();     // calculate minimal and maxium
  temp_detect();   // measure temperature and detect error
  switch (status) {
    case FAULT:
      // Add a readpin to eliminate FAULT
      digitalWrite(BMS_FAULT_PIN, HIGH);
      break;
    case WORK:
      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        if (vmax[current_ic] >= 4.25 || vmin[current_ic] <= 2.8) {
          status = FAULT;
        } else if (vmax[current_ic] >= 4.2) {
          work_loop();
          digitalWrite(BMS_FAULT_PIN, LOW);
        }
      }
      break;
    case CHARGE:
      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
        if (vmax[current_ic] >= 4.25 || vmin[current_ic] <= 2.8) {
          status = FAULT;
        } else if (vmax[current_ic] >= 4.12) {
          charge_loop();
          digitalWrite(BMS_FAULT_PIN, LOW);
        }
      }
      break;
    default:
      status = FAULT;
      break;
  }
}

void balance(double min_thr, double max_thr) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      if (abs(BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
              consvmin[current_ic]) <= 0.1) {
        Serial.print(abs(BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
                         consvmin[current_ic]));
        Serial.println(", stop discharge");
        stop_ic_discharge(i + 1, TOTAL_IC, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      } else if ((BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
                  consvmin[current_ic]) > 0.1) {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
                     consvmin[current_ic]);
        Serial.println(", dischage");
        set_ic_discharge(i + 1, current_ic, BMS_IC);
        LTC6811_wrcfg(TOTAL_IC, BMS_IC);
      } else if ((BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
                  consvmin[current_ic]) < -0.1) {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001 -
                     consvmin[current_ic]);
        Serial.println(", do nothing");
        // reset_vmin();
      }
    }
  }
}

void set_ic_discharge(
    int Cell,            // The cell to be discharged
    uint8_t current_ic,  // The subsystem of the selected IC to be discharge
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

void stop_ic_discharge(
    int Cell,            // The cell to stop discharging
    uint8_t current_ic,  // The subsystem of the selected IC to discharging
    cell_asic *ic        // A two dimensional array that will store the data
) {
  switch (Cell) {
    case 1:
      ic[current_ic].config.tx_data[4] =
          ic[current_ic].config.tx_data[4] & (0xFE);
      break;
    case 2:
      ic[current_ic].config.tx_data[4] =
          ic[current_ic].config.tx_data[4] & (0xFD);
      break;
    case 3:
      ic[current_ic].config.tx_data[4] =
          ic[current_ic].config.tx_data[4] & (0xFB);
      break;
    case 4:
      ic[current_ic].config.tx_data[4] =
          ic[current_ic].config.tx_data[4] & (0xF7);
      break;
    case 5:
      ic[current_ic].config.tx_data[4] =
          ic[current_ic].config.tx_data[4] & (0xEF);
      break;
    case 6:
      ic[current_ic].config.tx_data[4] =
          ic[current_ic].config.tx_data[4] & (0xDF);
      break;
    case 7:
      ic[current_ic].config.tx_data[4] =
          ic[current_ic].config.tx_data[4] & (0xBF);
      break;
    case 8:
      ic[current_ic].config.tx_data[4] =
          ic[current_ic].config.tx_data[4] & (0x7F);
      break;
    case 9:
      ic[current_ic].config.tx_data[5] =
          ic[current_ic].config.tx_data[5] & (0xFE);
      break;
    case 10:
      ic[current_ic].config.tx_data[5] =
          ic[current_ic].config.tx_data[5] & (0xFD);
      break;
    case 11:
      ic[current_ic].config.tx_data[5] =
          ic[current_ic].config.tx_data[5] & (0xFB);
      break;
    case 12:
      ic[current_ic].config.tx_data[5] =
          ic[current_ic].config.tx_data[5] & (0xF7);
      break;
    default:
      break;
  }
}

void calculate() {  // calculate minimal and maxium
  int8_t error = 0;
  uint32_t conv_time = 0;

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
}

void reset_vmin() {
  for (int i = 0; i < TOTAL_IC; i++) {
    consvmin[i] = vmin[i];  // Set up a costant Vminimum in case of the
                            // minumum become lower and lower
  }
}

void select(int ic, int cell) {
  int8_t error = 0;
  uint32_t conv_time = 0;

  wakeup_sleep(TOTAL_IC);
  conv_time = LTC6811_pollAdc();
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
  check_error(error);  // Check error to enable the function

  wakeup_sleep(TOTAL_IC);
  set_ic_discharge(cell, ic, BMS_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
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
  const float RT0 = 10000;  // The NTC resistance during 25 deg C
  const float RT1 = 32650;  // The NTC resistance during 0 deg C
  const float RT2 = 588.6;  // The NTC resistance during 100 deg C
  const float T0 = 298.15;  // The Kelvin during 25 deg C
  const float T1 = 273.15;  // The Kelvin during 0 deg C
  const float T2 = 378.15;  // The Kelvin during 105 deg C

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < 12; i++) {
      Serial.print(BMS_IC[current_ic].aux.a_codes[i]);
      Serial.print(", ");
    }
  }
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < 12; i++) {
      // GPIO->V
      // 10V/(5-V)= R = 10*exp(3984*(1/T-1/298.15))
      // Rt = R*EXP(B*(1/T1-1/T2))
      if (BMS_IC[current_ic].aux.a_codes[i] != 0) {
        float VoltageOut;
        float ROut;
        float beta;
        float Rx;
        float KelvinValue;

        beta = (log(RT1 / RT2)) / ((1 / T1) - (1 / T2));
        Rx = RT0 * exp(-beta / T0);

        VoltageOut = BMS_IC[current_ic].aux.a_codes[i] * 0.0001;

        ROut = THRES * VoltageOut /
               (THSourceVoltage - VoltageOut);  // current NTC resistance
        KelvinValue = (beta / log(ROut / Rx));

        BMS_IC[current_ic].aux.a_codes[i] =
            KelvinValue - 273.15;  // Kelvin to deg C
      }
    }
  }
  error_temp();
}

void error_temp() {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < 12; i++) {
      if (BMS_IC[current_ic].aux.a_codes[i] > 60) {
        Serial.println(
            F("************* Over maximum Temperature *************"));
        status = FAULT;
      }
      if (BMS_IC[current_ic].aux.a_codes[i] <= 0) {
        Serial.println(
            F("************* Temprature plug has gone *************"));
      }
    }
  }
}