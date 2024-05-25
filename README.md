[繁體中文](https://github.com/ncku-formula-racing/bms-firmware/blob/main/README_CH.md)

# Intro

[![hackmd-github-sync-badge](https://hackmd.io/2i-s6kk5SbGv-v5dF11x8w/badge)](https://hackmd.io/2i-s6kk5SbGv-v5dF11x8w)

Welcome to the accumulator team, you will learm how to compile and upload the firmware of BMS up to the control board of accumulator here  

# Prerequisite
* Arduino IDE
* Arduino Dual

# Environment
Before entering the topic, there's a few prerequisite knowledge to let you know. First of all, we can find the default library path inside the setting of Arduino IDE, no matter you are using Windows of Mac, the default path should be `home/Documents/Arduino` and under the folder `Arduino` is the `library` folder.  

As for the LTS series library of Anolog Device, you have to download their exclusive library called `LTSketchbook` from their official Github and go to the setting of your Arduino IDE to change the reference path of the library to it.  

However, in the scenario this time, we cannot apply the method above to access the functions of the LTS series library, since our inherited code and library had been modefied by our senior member so that it is either the version they used is so old or that they had changed the code in both the sample code and the library that we cannot access the API of LTS library by the regular way.  

To sum up, it is not only the source code that is inherited but also the library, which is quite horrible(so 張庭瑋 aka our savior suggest that we rebuild the whole stuff from scratch). But for now, we teach you the solution just in case.

## LTSketchbook

Basically you go to [this website](https://github.com/analogdevicesinc/Linduino) and follow the steps above  

## bms-firmware environment setup

Ok, back to the topic:  
All the file and documents that you need for setting up the environment are uploaded to [Github](https://github.com/ncku-formula-racing/bms-firmware/tree/Old_version), so the only thing you have to do is to either clone it to your local repository or download the zip and uncompress it, the point is the progresses that you have to execute after downloading the file.  

Start up your Arduino IDE and follow the step in [Environment](https://hackmd.io/@nckufs/ryvS1uIgA#Environment) to select the reference path of your library, after that, go back to the main page of your code and then try to click the `verify` buttom and see if it can be compiled properly or not. If you succeeded then it will be fine, but in case of you having any problem, leave a comment or go ask **簡誌加** or **張庭瑋**.  

![image](https://hackmd.io/_uploads/BJIMAK8xR.png)  
> The UI looks like this


![image](https://hackmd.io/_uploads/BJAuAt8xA.png)  
> You set your library path this way with a Windows  


![image](https://hackmd.io/_uploads/r1Rg15UxC.png)  
> The upper half if UI looks like this, the path may varies due to the existance of OneDrive  

# Epilogue
This is a simple tutorial for anyone who's new to the battery to understand the mechanism of how to operate BMS system.  
But it is just a start, our goal is to totally rewrite the whole program and then make the poject more easy to read and maintain, the current document will keep being updated  

Also, if there's any part that you don't understand, please don't be hesitate to leave a comment with your account in `HackMD`, as long as we notice your comment, we can improve the tutorial and make it better for anyone to understand.  
