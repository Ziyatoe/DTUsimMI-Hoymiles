DTU simulator for Hoymiles MI-Series Microinverter over RF (NRF24), on ESP8266 & nRF24L01+
-

Supports only ONE inverter!

Supported inverters, with serial number begin:
- 0x1020xxxxxx,0x1021xxxxxx
- 0x1040xxxxxx,0x1041xxxxxx
- 0x1060xxxxxx,0x1061xxxxxx

Hoymiles HM inverters are supported in this project:
https://github.com/lumapu/ahoy/

This software is a QUICK&DIRTY SW for debugging/controlling the Hoymiles inverters over RF, tested with:

- MI1500, 3 and 4 PV's. 
- MI600
- MI300
- TSUN800 (as MI600), 2 PV's
- should work with MI1200 also

Radio, CRC, part of Web based on the orig. SW from Hubi's earlier stage from this (https://github.com/hm-soft/Hoymiles-DTU-Simulation).

Project initiated here: https://www.mikrocontroller.net/topic/525778

Do not expect any quality from this SW!!! 

Issues can be reported on  https://www.mikrocontroller.net/topic/525778, but do not expect any answer 
immediately!!! 

As far as you know what you are doing, you can use this SW. 

READ THE THINGS BELOW BEFORE REQUEST SUPPORT:

- you need ArduinoIDE for this skecth! tested with arduino-ide 2.0.0-rc6 und 1.8.19
- you have to configure your Arduino-IDE for ESP8266
- you need to install nrf24 library, tested with RF24 lib Version:1.4.5
- keep all source files in same folder
- configure secrets.h
- configure settings.h


-----------------------------------------------------------------------------------------------------

Runs on ESP8266: wifi, mqtt, shows all data on serial monitor and on web site 

Wiring ESP8266 & nRF24L01+
-
https://github.com/lumapu/ahoy/blob/main/Getting_Started.md#things-needed

-----------------------------------------------------------------------------------------------------



For zeroexport, it needs a sep. mqtt connection with a topic "ImpExpW" to a Chint-DTSU, see mqtt.h .
I had to choose the topics so, because of an earlier implementation on
RS485 modbus DTUPro<>DTSU666 in my house. You can change it as you like!
If you change the topics, be sure change the PV numbers everywhere!

Define mostly everything in settings.h and secrets.h 

- Runs also as NRF24-Sniffer (adr 0x00aa, 0x0055, listen everything) if defined.
- Output on HTTP and serial monitor, if defined
- Controlling over several serial commands 
- sends all data to a mqtt broker, if defined

Please SEE the issues !!!

NO COMMERCIAL USE !!
--

>VERSION "V0.1.5"  
- using secrets.h for SSID,PW etc.
- stay on same RxCH until no Rx OR gets a rxtimeout
- fine tuning on zeroexport
- update timer for checkPV,zeroexport, webservice, mqtt
- Tx only if its daytime (only with Wifi, otherways always Tx)
- new mqtt id
- still issues with Rx-irq, no issues with Rx-polling

>VERSION "V0.1.6"
- fixed limiting the wr over WR_LIMITTED in settings.h
- more data moved to settings.h
- several enhancements

>VERSION "V0.1.7"
- new file "Globals.h"
- automatic recognition the inverter (MI) model,power and ports
- several changes for stability
- zeroexport with % of reated power OR absolute power

>VERSION "V0.1.9"
- no irq issues anymore
- significant modifications
- a lot of enhancements
- more serial commands for inverter, type 1 help on console
- mqtt reconnect
- JSON string to mqtt

>VERSİON "V0.1.9.1"
- changed max. power of one MI.port
