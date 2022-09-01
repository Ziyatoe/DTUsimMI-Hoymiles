/* die orig. SW ist vom Hubi, wurde von mir(Ziyat T.) für den MI-WR abgeaendert.
Getestet auf ESP8266/ArduinoUNO.
https://www.mikrocontroller.net/topic/525778
https://github.com/hm-soft/Hoymiles-DTU-Simulation
----------------------------------------
Alle Einstellungen sind in Settings.h UND secrets.h !!
----------------------------------------
*/
#ifndef __SETTINGS_H
#define __SETTINGS_H

#include <stdint.h>
#include <RF24.h>
#include <Pinger.h>
#include "secrets.h"    //<<<<<<<<<<<<<<<<<< put your secrets here

// Hardware configuration, CHECK THIS OUT with your board !!!!!!!!!!!!!!!!!!!!!!!!!!
// ESP8266 PIN Setting====================================================================================
#ifdef ESP8266
    #define RF1_CE_PIN  (D2) //(D2)
    #define RF1_CS_PIN  (D8) //(D8)
    #define RF1_IRQ_PIN (D3)
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
// ESP8266 PIN Setting====================================================================================


//=== DEFINE YOUR CONFIGURATION HERE =====================================================================
#define SER_BAUDRATE            (115200)
#define DEBUG                 // output all infos on serial monitor, comment it for no infos

bool WITHWIFI       = 1;      // wifi yes/no
bool WITHMQTT       = 1;      //do you need mqtt? if yes, may be you want to change the topic names in mqtt.h
bool ZEROEXP        = 0;      //zero export to the grid
int WR_LIMITTED     = 0;    //fixed limiting in Watt. if you dont want fixed limiting set it to NULL

// WR Config ..................................................................................
#define SerialWR      MY_MI_WR	// <<<<<<<<<<<<<<<<<<<<<<< define in secrets.h
bool MI300  = 0;     //<<<<<<<<<<<< choose which model of Hoymiles MI microinverter
bool MI600  = 0;     //choose this for TSUN TSOL-M800 also
bool MI1500 = 1;
#define NRofPV  3   //<<<<<<<<<<<< number of PV panels in use (connected) 1-4. (not the number of WR-ports)
                    //assume the ports are connected on order from 1 to 4
                    //we to know how many PVs are really connected to WR.
                    //if we know it, we dont have to wait of all ports!

// Webserver ..................................,,,,,,...........................................
IPAddress ROUTER = IPAddress(192,168,1,1);       // your routers IP???
#define WEBSERVER_PORT      80

// Time Server .............................,,,,................................................
#define TIMESERVER_NAME "pool.ntp.org"
//#define TIMESERVER_NAME "fritz.box"

// MQTT ........................................................................................
const char MQTTbroker[] = "192.168.1.11";
int        MQTTport     = 1883;
char       MQTTclientid[] = "MYDTU";
// END OF YOUR CONFIGURATION ===================================================================


//=== define your ADVANCED configuration========================================================

bool DEBUG_RCV_DATA = 0;      // output all rcv data on serial monitor
bool DEBUG_TX_DATA  = 0;      // output all tx data on serial monitor
bool CHECK_CRC      = 1;      //without crc, more data but may be wrong, must be checked
bool INTERRUPT      = 0;      //with or without interrupt
bool SNIFFER        = 0;      //as sniffer you just listen everything
bool ONLY_RX        = 0;      //nothing will be sent to inverter, only receive, if you have a dtu
//#define WITH_OTA              // mit OTA Support, also update der Firmware über WLan mittels IP/update

#define DEFAULT_RF_DATARATE     (RF24_250KBPS)  // Datarate
uint8_t PA_LEVEL    = RF24_PA_HIGH;  // <<<<<<<<<<<<<<<<<<<<<<< define PA LEVEL on NRF24

#define milisek 1000
#define IPServicesUPDATETICK   (15*milisek) //15sek
#define ZEXPUPDATETICK         (60*milisek) //60sek update zeroexport
#define FIXLIMITTICK        (ZEXPUPDATETICK*5) // 5 min to set the limit again
#define TXTIMER             700   //send request on every TXTIMER
#define TIMEOUTRXACK        (10*milisek) //10sek RxAck timeout
#define TIMERPVCHECK        (120*milisek) //60sek PV check timeout

// OTA ................................................................................................
#ifdef WITH_OTA
    // OTA Einstellungen
    #define UPDATESERVER_PORT   WEBSERVER_PORT+1
    #define UPDATESERVER_DIR    "/update"		// mittels IP:81/update kommt man dann auf die OTA-Seite
    #define UPDATESERVER_USER   "username_für_OTA"	// <<<<<<<<<<<<<<<<<<<<<<< anpassen
    #define UPDATESERVER_PW     "passwort_für_OTA"	// <<<<<<<<<<<<<<<<<<<<<<< anpassen
#endif

// globals --DO NOT CHANGE ===========================================================================
static String STARTTIME="xx";



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

// DTU&WR =====================================================================================
uint64_t WR1_RADIO_ID = Serial2RadioID (SerialWR);
#define DUMMY_RADIO_ID          ((uint64_t)0xDEADBEEF01ULL)
#define DTU_RADIO_ID            MY_DTU_PRO			// <<<<<<<<<<<<<<<<<<<<<<< secrets.h

char MIWHAT[10];

// zeroexport ===================================================================================
#define PVPOWER 350  //each PV
int MAXPOWER = NRofPV * PVPOWER;   //for zeroexport
int MINPOWER = int(MAXPOWER / 10); //watt  (10%) under this is the WR off
#define TOLERANCE 15 //watt


// WIFI ===============================================================================================
// PREFIXE dienen dazu, die eigenen WLans (wenn mehrere) vonfremden zu unterscheiden
// gehe hier davon aus, dass alle WLans das gleiche Passwort haben. Wenn nicht, dann mehre Passwörter hinterlegen
#define SSID_PREFIX1         MY_SSID			// <<<<<<<<<<<<<<<<<<<<<<< secrets.h
//#define SSID_PREFIX2         "wlan2-Prefix"		// <<<<<<<<<<<<<<<<<<<<<<< anpassen
#define SSID_PASSWORD        MY_WIFIPW			// <<<<<<<<<<<<<<<<<<<<<<< secrets.h

// Geo location ========================================================================================
#define  geoBreite  MY_BREITE				// <<<<<<<<<<<<<<<<<<<<<<< secrets.h
#define  geoLaenge  MY_LAENGE				// <<<<<<<<<<<<<<<<<<<<<<< secrets.h

static bool istTag = true;

#endif
