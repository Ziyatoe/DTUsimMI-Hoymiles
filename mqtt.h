/* die orig. SW ist vom Hubi, wurde von mir(Ziyat T.) für den MI-WR abgeaendert.
Getestet auf ESP8266/ArduinoUNO.
https://www.mikrocontroller.net/topic/525778
https://github.com/hm-soft/Hoymiles-DTU-Simulation
----------------------------------------
Alle Einstellungen sind in Settings.h UND secrets.h !!
----------------------------------------
*/
#include <Arduino.h>
#include <SPI.h>
#include "Settings.h"

#include <ArduinoMqttClient.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <Pinger.h>       // von url=https://www.technologytourist.com   
#else
#endif


static  uint8_t MQTT= 0;
float   GridPower   = 0;
// MQTT topics=================================================================
//I had to choose the topics so, because of an earlier implementation on
//RS485 modbus DTUPro<>DTSU666 in my house. You can change it as you like!
//If you change the topics, be sure change the PV numbers everywhere!
static char INVTOT_P[]              = "Mi01_totalW";
static char LIMIT_P[]               = "Limiting";
static char INV_P_PVNR[]            = "Mi01pvPower";
static char INV_UDC_PVNR[]          = "Mi01_UPv";
static char INV_IDC_PVNR[]          = "Mi01_IPv";
static char INV_Q_PVNR[]            = "Mi01EnergiePV";
static char INV_TEMP[]              = "WRtemp";
static char INV_STS_PVNR[]          = "Mi01StsPort";
static char INV_CONSUM_P[]          = "ConsumW";
static char DAY[]                   = "Day";
static char TIME[]                  = "Time";
static char INFO[]                  = "Error";    //this is used also for the info
static char GRID_P[]                = "ImpExpW";  //topic for reading the gridpower
// MQTT topics=================================================================

char ValueStr[20] = "";
char TopicStr[20] = "";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

void onMqttMessage(int messageSize) {
//----------------------------------------------------------------------------------------

  // we received a message, get the topic and contents
  uint8_t i=0;

  strcpy(TopicStr,(char *)mqttClient.messageTopic().c_str());
  while (mqttClient.available()) {
    ValueStr[i]=(char)mqttClient.read();
    i++;
  }  
//  DEBUG_OUT.printf("Received Topic: %s l:%i %s Watt\r\n",TopicStr,length,ValueStr);
  if ( strcmp(TopicStr,GRID_P ) ==0){ //true if = 0   Import/Export P Watt
      GridPower= atof(ValueStr);
  }

}//----------------------------------------------------------------------------------------

uint8_t setupMQTT(void){
//----------------------------------------------------------------------------------------
  if (!WITHMQTT)
    return 0;
  DEBUG_OUT.printf("Attempting to connect to the MQTT broker: %s\r\n",MQTTbroker);
  mqttClient.setId(MQTTclientid);
  if (!mqttClient.connect(MQTTbroker, MQTTport)) {
    DEBUG_OUT.printf("MQTT connection failed! Error code %i\r\n",mqttClient.connectError());
    MQTT=0;
  }
  else{
    DEBUG_OUT.printf("MQTT connected (null is ok) %i\r\n",mqttClient.connectError());
    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);
    if (ZEROEXP)
        DEBUG_OUT.printf("Subscribing to topics.. %s\r\n",GRID_P);
        // subscribe to a topic
        mqttClient.subscribe(GRID_P); //import export
    MQTT=1;
  }
  return MQTT;
}//----------------------------------------------------------------------------------------- 

