DTU simulator for Hoymiles MIx00and MI1x00 Microinverter over RF (NRF24)

This software is a QUICK&DIRTY SW for debugging/controlling the MI1000/1500 inverter over RF, tested with MI1500 and 3 PV's.

Based on the orig. SW from Hubi (https://github.com/hm-soft/Hoymiles-DTU-Simulation).

Project initiated here: https://www.mikrocontroller.net/topic/525778

Do not expect any quality from this SW!
As far as you know what you are doing, you can use this SW.

Runs on ESP8266: wifi, mqtt, on serial monitor and on web site, shows all data. 
Runs on Arduino: no wifi, serial monitor shows all data. arduino-ide 2.0.0-rc6 und 1.8.19

For zeroexport, needs a sep. mqtt connection to a Chint-DTSU, see mqtt.h .

Define mostly everything in settings.h and secrets.h

.runs also as NRF24-Sniffer (adr 0x00aa, 0x0055, listen everything)
.Channel  TX=allways hopping / RX=hopping after a timeout
.with/without CRC
.for ZeroExport: connect over MQTT with a DTSU
.output on serial monitor 
.several serial commands 

Please SEE the issues !!!

VERSION "V0.1.5"  
- using secrets.h for SSID,PW etc.
- stay on same RxCH until no Rx and gets a timeout
- fine tuning on zeroexport
- update timer for checkPV,zeroexport, webservice, mqtt
- Tx only if its daytime (only with Wifi, otherways always Tx)
- mqtt id
- still issues with Rx-irq, no issues with Rx-polling
