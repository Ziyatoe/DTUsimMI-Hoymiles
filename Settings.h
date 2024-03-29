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

#ifndef __SETTINGS_H
#define __SETTINGS_H

#include <stdint.h>
#include <RF24.h>
#include <Pinger.h>
#include "secrets.h"    //<<<<<<<<<<<<<<<<<< put your secrets here

// Hardware configuration, CHECK THIS OUT with your board !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ESP8266 PIN Setting==================================================================================================
#ifdef ESP8266
    #define RF1_CE_PIN  (D3) //(D2)
    #define RF1_CS_PIN  (D8) //(D8)
    #define RF1_IRQ_PIN (D4) ///D3)
//GPIO0  D3 -> IRQ
//GPIO2  D4 -> CE
//GPIO14 D5 -> SCK
//GPIO12 D6 -> MIso
//GPIO13 D7 -> MOsi
//GPIO15 D8 -> CSN
#else   //arduino PIN setting
  #define RF1_CE_PIN  (9)
  #define RF1_CS_PIN  (10)
  #define RF1_IRQ_PIN (2)
#endif
// ESP8266 PIN Setting==================================================================================================


//=== DEFINE YOUR CONFIGURATION HERE ===================================================================================
#define SER_BAUDRATE  (115200)
#define DEBUG                    // output all infos on serial monitor, comment it for no infos
static bool WITHWIFI       = true;      // wifi yes/no
static bool WITHMQTT       = true;      //do you need mqtt? if yes, may be you want to change the topic names in mqtt.h
static bool ZEROEXP        = false;     //zeroexport(zeropower)  to the grid
/*For zeroexport, it needs a sep. mqtt connection with a topic "ImpExpW", which is the power data,
  readed from a Chint-DTSU (smart meter), see mqtt.h.
  this "ImpExpW" must be readed over a RS485/modbus connection from a Chint-DTSU and must be published
  to the mqtt-broker trough a external device
  Be aware!
  DTSU(smart meter) Export power must be PLUS
  DTSU(smart meter) Import power must be MINUS
*/



static int WR_LIMITTED     = 0;         //watt, if not NULL, inverter is permanently limited around this power
                                 //if you DO NOT want permanently limiting the inverter, set it to NULL

// WR Config ..................................................................................
#define SerialWR  MY_MI_WR	// <<<<<<<<<<<<<<<<<<<<<<< define in secrets.h
uint8_t NRofPV = 0; //<<<<<<<<<<<< number of PV panels in use (connected) 1-4. (not the number of WR-ports)
                    //assume the ports are connected in order from port1 to port4
                    //we have to know how many PVs are REALLY connected to the WR.
                    //if we know it, we dont have to wait of the not connected ports!
                    //if NRofPV is set to NULL, it will be determined automatically in GetMIModel() over the MI models
                    //number of ports

// Webserver ..................................,,,,,,...........................................
IPAddress ROUTER = IPAddress(192,168,1,1);       // your routers IP???
#define WEBSERVER_PORT      80

// Time Server .............................,,,,................................................
#define TIMESERVER_NAME "pool.ntp.org"
//#define TIMESERVER_NAME "fritz.box"

// MQTT ........................................................................................
static const char MQTTbroker[] = "192.168.1.11";
static int        MQTTport     = 1883;
static char       MQTTclientid[] = "MYDTU";

//define all single topics in mqtt.h or go with JSON below
//#define SENDJSON  // <==================== send Tasmota compatible MQTT JSON instead of single topics
#ifdef SENDJSON
  static String JSON_TOPIC  = String(MQTTclientid)+"/SENSOR";
#endif
// END OF YOUR CONFIGURATION ===========================================================================================
//
//
//
//
//    DO YOU KNOW WHAT YOU ARE DOING, DONT'YOU????
//=== define your ADVANCED configuration================================================================================

static bool DEBUG_RX_DATA = false;      // output all rcv data on serial monitor
static bool DEBUG_TX_DATA  = false;      // output all tx data on serial monitor
static bool CHECK_CRC      = true;       // without crc, we get more data but they may be wrong, must be checked
static bool INTERRUPT      = false;      // with or without interrupt
static bool SNIFFER        = false;      // as sniffer you just listen everything
static bool ONLY_RX        = false;      // nothing will be sent to inverter, only receive data, if you have a dtu in parallel;-)
#define WITH_OTA                  // OTA Support, update Firmware IP:81/update

#define DEFAULT_RF_DATARATE     (RF24_250KBPS)  // Datarate, dont touch!!
uint8_t PA_LEVEL    = RF24_PA_LOW;   // define PA LEVEL on NRF24  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CHECK THIS OUT
                                     // if you do not receive anything, you might play with it
                                     // power level of NRF24 are: RF24_PA_LOW, RF24_PA_HIGH or RF24_PA_MAX
                                     // or choose 3-5 from serial commands on the console

//DO NOT CHANGE ANY OF THIS WITHOUT KNOWING WHAT !!!!
#define milisek 1000
#define IPServicesUPDATETICK   (15*milisek)    //15sek
#define ZEXPUPDATETICK         (30*milisek)    //60sek update zeroexport
#define FIXLIMITTICK        (ZEXPUPDATETICK*8) // ca.5 min to set the fix limit again, just don't rely on chance
#define TXTIMER             250                //send request on every TXTIMER, better you do not change it
#define TIMEOUTRXACK        (20*milisek)      //10sek RxAck timeout  //todo changed 20
#define TIMERPVCHECK        (60*milisek)      //60sek PV check timeout
#define AFTERHH 17                            //hh, after this hour, the inverter wont be under MINPOWER

// zeroexport .................................................................................
bool LIMITABSOLUT = true;               //true : inverter will be limited with absolut power number, like 525 watt
                                        //false: inverter will be limited with % of rated power, like 13% of MAXPOWER

#define TOLERANCE 16                   //watt, tolerance on grid power, above/bellow we do zeroexport

// OTA ........................................................................................
#ifdef WITH_OTA
    // OTA Einstellungen
    #define UPDATESERVER_PORT   WEBSERVER_PORT+1
    #define UPDATESERVER_DIR    "/update"		    // IP:81/update OTA site
    #define UPDATESERVER_USER   "OTA"	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CHECK THIS OUT
    #define UPDATESERVER_PW     "OTA"	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CHECK THIS OUT
#endif

// DTU&WR =====================================================================================
union longlongasbytes {
  uint64_t ull;
  uint8_t bytes[8];
};

uint64_t Serial2RadioID (uint64_t sn) {
//----------------------------------
  longlongasbytes llsn;
  longlongasbytes res;
  llsn.ull = sn;
  res.ull = 0;
  res.bytes[4] = llsn.bytes[0];
  res.bytes[3] = llsn.bytes[1];
  res.bytes[2] = llsn.bytes[2];
  res.bytes[1] = llsn.bytes[3];
  res.bytes[0] = 0x01;
  return res.ull;
}

uint64_t WR1_RADIO_ID = Serial2RadioID (SerialWR);
#define DUMMY_RADIO_ID          ((uint64_t)0xDEADBEEF01ULL)
#define DTU_RADIO_ID            MY_DTU_PRO			// <<<<<<<<<<<<<<<<<<<<<<< secrets.h



// WIFI ===============================================================================================
// PREFIXE dienen dazu, die eigenen WLans (wenn mehrere) vonfremden zu unterscheiden
// gehe hier davon aus, dass alle WLans das gleiche Passwort haben. Wenn nicht, dann mehre Passwörter hinterlegen
#define SSID_PREFIX1         MY_SSID			// <<<<<<<<<<<<<<<<<<<<<<< secrets.h
//#define SSID_PREFIX2         "wlan2-Prefix"	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CHECK THIS OUT
#define SSID_PASSWORD        MY_WIFIPW			// <<<<<<<<<<<<<<<<<<<<<<< secrets.h

#define REFRESH 30  //http REFRESH
// Geo location ========================================================================================
#define  geoBreite  MY_LATITUDE				// <<<<<<<<<<<<<<<<<<<<<<< secrets.h
#define  geoLaenge  MY_LONGITUDE			// <<<<<<<<<<<<<<<<<<<<<<< secrets.h
#define  TIMEOFFSET 0                        //winter/sommer



#endif //__SETTINGS_H
