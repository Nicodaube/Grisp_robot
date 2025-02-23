# LilyGo robot software Installation (Ubuntu)

## Arduino IDE Installation

First download the Arduino IDE AppImage from [this](https://www.arduino.cc/en/software) page

While the file is downloading, you can install universe and libfuse2 :

```bash
sudo add-apt-repository universe
sudo apt install libfuse2
```

Once the AppImage is downloaded, search for the file, then :
* Right click
* Properties
* Make executable

You can now run Arduino IDE.

## Installing the LoRa librairy

Normally you can find it on the Sketch -> include librairy -> manage librairy dashboard but I couldn't find the right librairy. Instead, go to the [LoRa repository](https://github.com/sandeepmistry/arduino-LoRa) and download the LoRa.h and Lora.cpp files located in src in the same folder as your LilyGO.ino file.


## Uploading Code onto LilyGO
Grant yourself the permissions to write to the LilyGO
:warning: your computer will reboot
```bash
sudo usermod -aG dialout $(whoami)
sudo usermod -aG uucp $(whoami)
newgrp dialout
newgrp uucp
sudo reboot
```

Relaunch Arduino IDE and upload the code.
It should work.