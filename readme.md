DTU simulator for Hoymiles MI Microinverter over RF (NRF24)
-

Supports only ONE inverter!

Supported inverters, with serial number begin:
- 0x1021xxxxxx
- 0x1041xxxxxx
- 0x1061xxxxxx

This software is a QUICK&DIRTY SW for debugging/controlling the Hoymiles inverters over RF, tested with:

- MI1500, 3 PV's. 
- MI600
- TSUN800 (as MI600), 2 PV's
- should work with MI300, MI1200 also

Based on the orig. SW from Hubi's earlier stage from this (https://github.com/hm-soft/Hoymiles-DTU-Simulation).

Project initiated here: https://www.mikrocontroller.net/topic/525778

Do not expect any quality from this SW!!! 

Issues can be reported on  https://www.mikrocontroller.net/topic/525778, but do not expect any answer 
immediately!!! 

As far as you know what you are doing, you can use this SW.
-
You need ArduinoIDE for this skecth!

Runs on ESP8266: wifi, mqtt, shows all data on serial monitor and on web site 

Wiring https://github.com/lumapu/ahoy/tree/main/tools/esp8266#wiring-things-up
-

Runs on Arduino: no wifi, serial monitor shows all data. arduino-ide 2.0.0-rc6 und 1.8.19

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
