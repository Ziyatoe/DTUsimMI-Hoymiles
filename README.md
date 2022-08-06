# DTUsimMI
DTU simulator for Hoymiles MI Microinverter

This software is a QUICK&DIRTY SW for debugging the MI inverter, tested with MI1500 and 3 PV's.
Do not expect any quality!
As far as you know what you are doing, you can do everything with this sw.
Runs on ESP8266: wifi, mqtt, serial monitor and web site shows all data.
Runs on Arduino: no wifi, serial monitor shows all data.
For zeroexport, needs a mqtt connection to a Chint-DTSU, see mqtt.h .

Im settings.h sind einzustellen:
- Laeuft als Sniffer, wenns definiert (adr 0x00aa, 0x0055, hört alles 
mit).
- Channel hopping auch beim RX
- Mit/Ohne Interrupt,CRC zum Testen
- hat MQTT (auf meine Art) wenn Wifi, bekommt SmartMeterPower über MQTT
- kann über Serielle-SS gesteuert werden:
  serial commands:
  1: help 2:Status 3:PA_LOW 4:PA_HIGH 5:PA_MAX 6:Sniffer 7:ZeroEx 8:OnlyRX 9:ShowTX 10:Wifi 11:CRC 20:WRinfo 21:Gongfa
