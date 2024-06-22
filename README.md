# Outline 
[TOC]
# Introduction
This is the document for anyone who wants to know how the BMS works to understand the basic concept of our BMS system.  

In this document, you will know:  
* The power IC: LTC6811 
* The evaluation board and the IDE we are currently using for development
* The architecture and concept of our code
    * The source of our functions
    * Fault conditions
    * Future prospects
* Maintenance guideline

# Body
## LTC6811
The power IC on our slave board of BMS, for more information, please look up at these [sites](https://www.analog.com/en/products/ltc6811-1.html) and [datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6811-1-6811-2.pdf).  
![image](https://hackmd.io/_uploads/ryQKaSE8C.png)  

## Board and IDE
First of all, the evaluation board we are currently using for our master in the daisy chain system is `Arduino Due`.

![image](https://hackmd.io/_uploads/rJBzuHNLC.png)  

It is powered by `ATSAM3X8E` chip, which is an ARM based MCU. For more information, you can look up for it in its [datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf).  

As for the IDE, we are currently using `Arduino IDE` for the general development for out program. Since `LTC6811` has its correspond library on Arduino platform, we can easily access to those functions and APIs to manipulate the behavior of the ICs. BTW, the library is called [Linduino](https://github.com/analogdevicesinc/Linduino). If you want to know how to set up **Linduino** library for your Arduino IDE, you can go look that up at the [previous document](https://hackmd.io/@nckufs/ryvS1uIgA) I've written talking about the environment set up for development.  

## Architecture
In our code, it can be divided into serveral parts.  
* Includes
* Macro definition
* Functions declaration
* Variables setup
* **void setup()**
* **void loop()**
* Functions implementation

Since parts and parts are distinctly seperated, the structure of the whole program is quite easy to read and understand, which is also a part of the objective of the purpose of rewriting this project.  

In the next part, I am going to tell you about where the origin of the code comes from, the fault conditions of the BMS, and some future prospects that we hope to achieve some day.  

### Code source
We didn't make up the whole program from nothing by ourselves. Instead, we did refer to the official sample code written by **Linear Technology**, which is the developer company of LTC6811.  

For the usage of **LTC6811**, we paticularly choose the sample code of board `DC2259` to found the basic structure of our program. Those basic setups include:  
* Includes
* Some macros
* Most of the variables
* First few lines in **void setup()**
* Basic behavior functions such as setting configurations, reading datas and discharging

The whole program is expanded around the foundation of this sample code. Of course we did alot of deletion with the features we don't need, so the code itself doesn't look similar for the first glance, it does however inherit alot of features from the sample code at the end of the day.  

If you are interested with its origin, [here's the link](https://github.com/analogdevicesinc/Linduino/blob/master/LTSketchbook/Part%20Number/6000/6811/DC2259/DC2259.ino) for the github page of the sample code, which belongs to the official library of **Linear Technology**: **Linduino** (Which is already mentioned before above).  

### Fault Conditions
Now let's talk about fault conditions.  

In order to protect our driver's safety as well as our precious batteries, we need to know if the condition of our battery modules are fine or not. The way we know whether the batteries is ok is by measuring the voltage of each battery cell. By monitoring the voltage of the battery, we get to know some information by corresponding the voltages to the table that indicates the relationship between the volage and the status of the battery which is included in the documents of the spec of the battery cells.  

So here's the conditions that trigger **FAULT**:  
* Battery over charged
* Battery over discharged
* Battery over heated
* Battery disconnected
* Temperature sensors disconnected

Theoretically, if the battery doesn't violate any condition above, both the battery and the driver would be safe (hopefully). **If you find out anything is missing, please don't be hesitate to leave a comment**, we will be very appriciated.  

### Objective for now to improve
Are we perfect yet?  

**Definitely not, not at all**

So here's a list that I've made to improve someday in the future. Some of them are half-done already but still not properly functioning, some of them haven't even start a scratch yet. So if you are a lucky person who is going to take over my job and further improve the BMS, you can start with these objective, good luck!  

- [ ] Different voltage thresholds for different mode
- [ ] Timer trigger tasks
- [ ] Improve the flow of balancing
- [ ] Eliminate reduntant variables, comments, and serial prints
- [ ] Add the function to read the config regs and cmd regs
- [ ] Add the function of low pass filter (for the purpose of not getting fault so easily)
- [ ] (Ultimate) Make the balancing behavior "Active cell balancing"

## Maintenance guideline
