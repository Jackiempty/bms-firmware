# BMS Firmware

## Architecture

## Fault Condition

1. Maximum check: if any segment has cell's voltage > 4.3V by comparing global array vmax with 4.3, and vmax is calculated both in fcn voltage_update & fcn starttowork(call another fcn run_command(92))
2. Minimum check: if any segment has cell's voltage <2.5V by comparing global array vmin with 2.5, and vmin is calculated both in fcn voltage_update & fcn starttowork(call another fcn run_command(92))
3. error_average
    * Fcn error_average check if any segment's vmax>  cells' avg. voltage in the same segment, by compare global array vmax with avgvol.Global array  vmax & avgvol are calculate
4. error_difference
    * Fcn error_difference check if any segment's max voltage difference >0.65V, by compare vmax-vmin of each segment with 0.65, global array vmin & vmax are calculated in fcn voltage_update  & fcn starttowork(call another fcn run_command(92))
5. error_current
    * *no comment 


:::spoiler source code
```c
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
          //          Serial.println(F("Over maximum Temperature !"));
          //          shutdown_status = false;
          emergency_error[5] = 1;
        }
        if (BMS_IC[current_ic].aux.a_codes[i] <= 0) {
          //          Serial.println(F("Temprature plug has gone !"));
          //          shutdown_status = false;
          emergency_error[6] = 1;
        }
      }
    }
  }
}
```
:::


## Objective for now to improve
- [ ] Different voltage thresholds for different mode
- [ ] Timer trigger tasks
- [ ] Improve the flow of balancing
- [ ] Eliminate reduntant variables, comments, and serial prints
- [ ] Add the function to read the config regs and cmd regs
- [ ] Add the function of low pass filter (for the purpose of not getting fault so easily)
- [ ] (Ultimate) Make the balancing behavior "Active cell balancing"


## Charging strategy
