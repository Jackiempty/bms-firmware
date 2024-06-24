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

# LTC6811
The power IC on our slave board of BMS, for more information, please look up at these [sites](https://www.analog.com/en/products/ltc6811-1.html) and [datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6811-1-6811-2.pdf).  
![image](https://hackmd.io/_uploads/ryQKaSE8C.png)  

# Board and IDE
First of all, the evaluation board we are currently using for our master in the daisy chain system is `Arduino Due`.

![image](https://hackmd.io/_uploads/rJBzuHNLC.png)  

It is powered by `ATSAM3X8E` chip, which is an ARM based MCU. For more information, you can look up for it in its [datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf).  

As for the IDE, we are currently using `Arduino IDE` for the general development for our program. Since `LTC6811` has its correspond library on Arduino platform, we can easily access to those functions and APIs to manipulate the behavior of the ICs. BTW, the library is called [Linduino](https://github.com/analogdevicesinc/Linduino). If you want to know how to set up **Linduino** library for your Arduino IDE, you can go look that up at the [previous document](https://hackmd.io/@nckufs/ryvS1uIgA) I've written talking about the environment set up for development.  

# Architecture
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

## Code source
We didn't make up the whole program from nothing by ourselves. Instead, we did refer to the official sample code written by **Linear Technology**, which is the developer company of LTC6811.  

For the usage of **LTC6811**, we paticularly choose the sample code of board `DC2259` to found the basic structure of our program. Those basic setups include:  
* Includes
* Some macros
* Most of the variables
* First few lines in **void setup()**
* Basic behavior functions such as setting configurations, reading datas and discharging

The whole program is expanded around the foundation of this sample code. Of course we did alot of deletion with the features we don't need, so the code itself doesn't look similar for the first glance, it does however inherit alot of features from the sample code at the end of the day.  

If you are interested with its origin, [here's the link](https://github.com/analogdevicesinc/Linduino/blob/master/LTSketchbook/Part%20Number/6000/6811/DC2259/DC2259.ino) for the github page of the sample code, which belongs to the official library of **Linear Technology**: **Linduino** (Which is already mentioned before above).  

## Fault Conditions
Now let's talk about fault conditions.  

In order to protect our driver's safety as well as our precious batteries, we need to know if the condition of our battery modules are fine or not. The way we know whether the batteries is ok is by measuring the voltage of each battery cell. By monitoring the voltage of the battery, we get to know some information by corresponding the voltages to the table that indicates the relationship between the volage and the status of the battery which is included in the documents of the spec of the battery cells.  

So here's the conditions that trigger **FAULT**:  
* Battery over charged
* Battery over discharged
* Battery over heated
* Battery disconnected
* Temperature sensors disconnected

Theoretically, if the battery doesn't violate any condition above, both the battery and the driver would be safe (hopefully). **If you find out anything is missing, please don't be hesitate to leave a comment**, we will be very appriciated.  

## Objective for now to improve
Are we perfect yet?  

**Definitely not, not at all**  

So here's a list that I've made to improve someday in the future. Some of them are half-done already but still not properly functioning, some of them haven't even started a scratch yet. So if you are a lucky person who is going to take over my job and further improve the BMS, you can start with these objective, good luck!  

- [ ] Different voltage thresholds for different mode
- [ ] Timer trigger tasks
- [ ] Improve the flow of balancing
- [ ] Eliminate reduntant variables, comments, and serial prints
- [ ] Add the function to read the config regs and cmd regs
- [ ] Add the function of low pass filter (for the purpose of not getting fault so easily)
- [ ] (Ultimate) Make the balancing behavior "Active cell balancing"

# Maintenance guideline
In this part, I am going to talk about some cryterias when you are or by any mean going change any part of the code. To make the maintenance easy for anyone who is new in coding, I'm going to set some rules for you to follow so that the code can be easy to read and understand for the next maintainer and so on.  

* Seperate funtions' declarations and implementations
* **Don't** touch the library (which in this case is **LTSketchbook**)
* Use **clang-format** to unify the format of the code
* Use **Git** to do the version management
* Make any commitment with following **[this guideline](https://cbea.ms/git-commit/)**

In case you don't understand the detail of the cryterias, I will do some explaination for you.  

## Declaration and Defination
If you are familiar with the principles of writing a C code program, you may know that we typically declare the name and the type of the functions at the beginning of the program and then further define them after the main function, otherwise the compiler will jump errors telling that you are not writing the code in the right way.  

However, in Arduino IDE, the compiler is quite smart that you can still compile the program with or without following the mentioned principle, so many people would eventually forgot to follow it for the sake of the format of the program.  

Hence, I am telling you right now that I require you to follow this rule properly, else I will find you and condemn you harshly (Hopefully emotionally damage you).  

BTW, you may notice that I seperate all functions and variables with the classification of **"Stock"** and **"Custom"**, that's because some of the functions and varibles are directly inheritted from the sample code of LTC6811, and to even make that clear enough until where the boundary of the sample code and my own implementation is, I seperate these two parts with some commentary, to keep you noted.  

## About the library
**Do not** even think about make changes on the library  

So, here's the story.  

Before I rewrite the whole program, there's no one maintaining the code and no one even trying to understand what the code does and just use it without any concern 'cause that's what our senior teammates just did before. When I took over the project, I found out that the original library didn't work on our **"inheritted"** code because our senior teammates also made changes on the library, and it caused not only our main program but also the library become a black box for us to access since we cannot just understand what the main program does by just refering to the sample code.  

After this event, for the sake of simplicity of maintenance, we decided to prohibit any form of modification on the library.  

You might ask what we do when we want to do something that the library can do but not exactly in the way we want and we want to change the library. Let me just tell you, we just do whatever we want in the main program in the form of making it another function, even if it is originally done in the funcions of the library.  

## What is clang-format?
If a project is maintaining by more than a few people, in will be unpreventable that each of the programmer has their own coding style, and if a project is mixed with too many coding styles, it would be too messy to read.  

To prevent such things from happening, there's a tool called **"clang-format"** for developer to unify the format of a project so that after formatting, the format will be the same no matter who just changed it. For more information, you can look that up in [this site](https://clang.llvm.org/docs/ClangFormat.html).  

## Github and commit message
No further explaination, go look that up by yourself, since that's the least simple prerequisite for any programmer.  

# Operation guideline
