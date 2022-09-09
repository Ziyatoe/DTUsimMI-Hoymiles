/*
This software is a QUICK&DIRTY SW for debugging/controlling the Hoymiles inverters over RF(NRF24)
Based on the orig. SW from Hubi's earlier stage from his (https://github.com/hm-soft/Hoymiles-DTU-Simulation)
recoded and expanded  for the Hoymiles microinverter family MI, by Ziyat T.

Project initiated here: https://www.mikrocontroller.net/topic/525778
Do not expect any quality from this SW!!!

------------------------------------------------------------------------------------------------------------------------
Configuration are  in Settings.h and secrets.h !!
------------------------------------------------------------------------------------------------------------------------
*/
#ifndef __MQTT_H
#define __MQTT_H

#include <Arduino.h>
#include <SPI.h>
#include "Settings.h"

#include <ArduinoMqttClient.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <Pinger.h>       // von url=https://www.technologytourist.com   
#else
#endif



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
#ifdef SENDJSON
  String  GRID_PSTR                   = String(MQTTclientid)+"/ImpExpW";  //topic for reading the gridpower
  const char *GRID_P                  = GRID_PSTR.c_str();
#else
  static char GRID_P[]                = "ImpExpW";  //topic for reading the gridpower
#endif

// MQTT topics=================================================================

static char ValueStr[20] = "";
static char TopicStr[20] = "";
static bool SmartMeterOk = false;
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
      SmartMeterOk=true;
      }

}//----------------------------------------------------------------------------------------

bool setupMQTT(void){
//----------------------------------------------------------------------------------------
  bool isMQTT = false;
  if (!WITHMQTT)
    return false;
  DEBUG_OUT.printf("[MQTT] Attempting to connect to the broker: %s\r\n",MQTTbroker);
  mqttClient.setId(MQTTclientid);
  if (!mqttClient.connect(MQTTbroker, MQTTport)) {
    DEBUG_OUT.printf("[MQTT] connection failed! Error code %i\r\n",mqttClient.connectError());
    isMQTT = false;
  }
  else{
    DEBUG_OUT.printf("[MQTT] connected (null is ok) %i\r\n",mqttClient.connectError());
    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);
    if (ZEROEXP)
        DEBUG_OUT.printf("[MQTT] Subscribing to topics.. %s\r\n",GRID_P);
        // subscribe to a topic
        mqttClient.subscribe(GRID_P); //import export
    isMQTT = true;
  }
  return isMQTT;
}//----------------------------------------------------------------------------------------- 

#endif