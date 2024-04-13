# Intro

[![hackmd-github-sync-badge](https://hackmd.io/2i-s6kk5SbGv-v5dF11x8w/badge)](https://hackmd.io/2i-s6kk5SbGv-v5dF11x8w)

歡迎你來到電池組，在這裡我要教你如何建立可以編譯並上傳 BMS 韌體的程式的環境  

# Prerequisite
* Arduino IDE
* Arduino Dual

# Environment
Before entering the topic, there's a few prerequisite knowledge to let you know. First of all, we can find the default library path inside the setting of Arduino IDE, no matter you are using Windows of Mac, the default path should be `home/Documents/Arduino` and under the folder `Arduino` is the `library` folder.  


而如果要能夠操作 Anolog Device 的 LTS 系列函式庫的話，則需要上到官方的 Github 去下載他們專屬的函式庫`LTSketchbook`並到 Arduino IDE 的設定裡面去手動將函式庫的參考路徑改成這個資料夾  

然而，本次的環境架設無法將這兩個方法套用，因為我們的祖傳程式碼貌似被大學長們魔改過，所以包括 Arduino 的原生函式庫和 LTS 的官方函式庫在內，都有被修改過，要不就是現有版本跟祖傳的版本差異大到有些 API 已經不存在，所以在祖傳程式碼裡面所調用的 API 有些是學長自己加的或是跟原本不一樣的，導致你必須要將函式庫連同原始程式碼一起祖傳，是個非常母湯的做法 (所以我們的張庭瑋 aka 救世主說要重寫)，但目前還是得將這個現狀個操作方法記錄起來  

## LTSketchbook

基本上就是去[這個網站](https://github.com/analogdevicesinc/Linduino)然後按照上面的方法做就可以了  

## bms-firmware environment setup

說了那麼多，回到正題：  
所有架設環境的相關檔案都已經上傳到 [Github](https://github.com/ncku-formula-racing/bms-firmware) 了，所以你只要把他 clone 下來或是直接下載 zip 檔解壓縮後就可以使用，重點是下載下來之後有哪些步驟需要執行。  

首先，看你要不要把祖傳的 library 檔案移到`home/Documents/`去，這樣就可以統一所有文件的位置，將來不管是要切回去原本的`home/Documents/Arduino`還是要再切回來都比較好找到，因為就在同一個位置。  

打開 Arduino IDE 然後按照 [Environment](https://hackmd.io/@nckufs/ryvS1uIgA#Environment) 這邊的方式去選擇函式庫的參考路徑，最後回到程式碼的主頁試著點左上方的勾勾編譯看看，如果成功的話就沒問題了，如果有問題的話就問**張庭瑋**或**簡誌加**吧。  

![image](https://hackmd.io/_uploads/rkq5oYIe0.png)  
> Mac 的 Arduino IDE 要從這邊設定函式庫參考路徑  


![image](https://hackmd.io/_uploads/BJIMAK8xR.png)  
> 介面長這樣  


![image](https://hackmd.io/_uploads/BJAuAt8xA.png)  
> Windows 的 Arduino IDE 要從這邊設定函式庫參考路徑  


![image](https://hackmd.io/_uploads/r1Rg15UxC.png)  
> 介面的上半部長這樣，可以看到參考路徑，因為有 OneDrive 的關係，每個人有可能不太一樣  

# Epilogue
This is a simple tutorial for anyone who's new to the battery to understand the mechanism of how to operate BMS system.  
But it is just a start, our goal is to totally rewrite the whole program and then make the poject more easy to read and maintain, the current document will keep being updated  

Also, if there's any part that you don't understand, please don't be hasitate to leave a comment with your account in `HackMD`, as long as we notice your comment, we can improve the tutorial and make it better for anyone to understand.  
