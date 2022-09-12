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

#define VERSION "V0.1.9"


#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <RF24_config.h>
//#include "printf.h"
#include "Settings.h"
#include "CircularBuffer.h"
#include "hm_crc.h"
#include "hm_packets.h"
#include "Debug.h"
#include "Sonne.h"
#include "ModWebserver.h"
#include "Globals.h"

#ifdef ESP8266
  #include "wifi.h"
  #include "mqtt.h"
#else
 float GridPower=0; //arduino has no mqtt, we just say grid is null
#endif

#ifdef ESP8266
    #define DISABLE_EINT noInterrupts()
    #define ENABLE_EINT  interrupts()
#else     // für AVR z.B. ProMini oder Nano 
  #define DISABLE_EINT EIMSK = 0x00
  #define ENABLE_EINT EIMSK = 0x01
#endif


#define RF_MAX_ADDR_WIDTH       (5)
#define MAX_RF_PAYLOAD_SIZE     (32)
#include "NRF24_sniff_types.h"


#ifdef ESP8266
  #define PACKET_BUFFER_SIZE      (30)
#else
  #define PACKET_BUFFER_SIZE      (20)
#endif

static HM_Packets     hmPackets;
static uint64_t       UpdateTxMsgTick=0;
static uint64_t       UpdateIPServicesTick=0;
static uint64_t       UpdateZeroExpTick=0;

static uint64_t RxAckTimeOut = 100;   //milli sek.; search RxCH faster at the begin
static uint64_t timeLastPacket = millis();
static uint64_t timeLastRxAck    = RxAckTimeOut*4; //at the start
static uint64_t timeCheckPV = 100;    //milli sek.; search PV's faster at the begin
// Set up nRF24L01 radio on SPI bus plus CE/CS pins
// If more than one RF24 unit is used the another CS pin than 10 must be used
// This pin is used hard coded in SPI library

static RF24 radioDTU (RF1_CE_PIN, RF1_CS_PIN);
static NRF24_packet_t bufferData[PACKET_BUFFER_SIZE];
static CircularBuffer<NRF24_packet_t> packetBuffer(bufferData, sizeof(bufferData) / sizeof(bufferData[0]));
static Serial_header_t SerialHdr;


static uint16_t lastCRC;
static uint16_t crc;

static uint8_t channels[] = {3,23, 40, 61, 75};    //{1, 3, 6, 9, 11, 23, 40, 61, 75}
//2400 to 2525 MHz (MegaHz). The nRF24L01 channel spacing is 1 Mhz which gives 125 possible channels numbered 0 .. 124
static uint8_t TxChId = 0;                         // fange mit 3 an
static uint8_t RxChId = 0;                         // fange mit 3 an
static uint8_t TxCH   = channels[TxChId];
static uint8_t RxCH   = channels[RxChId];
static const char BLANK = ' ';
static const String LON  = "\033[1m";
static const String LOFF = "\033[0m";
static bool isRxIrq = false;
static char CHANNELNAME_BUFFER[15]="";
static int SerCmd = 0;
static char cStr[100];
static bool DataOK = false;
static uint8_t pvCnt[4]={0,0,0,0};
static float TotalP[5]={0,0,0,0,0}; //0 is total power, 1-4 are 4 PV power
static bool isOnTx = false;
#ifdef ESP8266
  #include "ModWebserver.h"
  //#include "Sonne.h"
#endif


static uint8_t sendBuf[MAX_RF_PAYLOAD_SIZE]; //MI-WR TxData
static char SerialIn[10]="";
static bool TxLimitSts = false; //quiet at the begin
static uint16_t Limit=0; //zeroexport power limit in %

typedef struct MIWR_t{ // define MI type
  char name[15];
  uint8_t NrPorts;
  int portP;
  };
#define MI300  0
#define MI600  1
#define MI1500 2
static MIWR_t MItype[3]={{"MI300",1,300}, // define MI type [NRPV,Modell,PortPower], bu sure of index!!
                         {"MI600",2,300},
                         {"MI1500",4,375}};
static uint8_t WhichMI = 0;//index inververter model
static int MAXPOWER = 0;
static int MINPOWER = 0;
static int MIportPower=0;
static byte FACTOR = 3; //todo zeroexport tuning


#ifdef ESP8266
char * getChannelName (uint8_t i) {
//------------------------------------------------------------------------------------------------
  memset (CHANNELNAME_BUFFER, 0, sizeof(CHANNELNAME_BUFFER));
  strcpy (CHANNELNAME_BUFFER, CHANNEL_NAMES[i]);
  //itoa (i, CHANNELNAME_BUFFER, 10);
  return CHANNELNAME_BUFFER;
}//-----getChannelName-----------------------------------------------------------------------------
#endif

inline static void dumpData(uint8_t *p, int len) {
//------------------------------------------------------------------------------------------------
  while (len--){
    if (*p < 16)
      DEBUG_OUT.printf("%c",'0');
    DEBUG_OUT.printf("%x",*p++);
  }
  DEBUG_OUT.printf("%c",BLANK);
}//----dumpData---------------------------------------------------------------------------------------------------------

#ifdef ESP8266
  IRAM_ATTR
#endif

bool RFRxPacket(void){
//---------------------------------------------------------------------------------------------------------------------
 static uint16_t lostPacketCount = 0;
 uint8_t pipe;
 bool sts=false;

// Loop until RX buffer(s) contain no more packets.
  while (radioDTU.available(&pipe)) {
    radioDTU.stopListening();//:::::::::::::::0

    //DEBUG_OUT.printf("RFRxPacket someting on pipe %i", pipe);
    if (!packetBuffer.full()) {
      //DEBUG_OUT.printf(" reading %i", i++);
      NRF24_packet_t *p = packetBuffer.getFront();
      if (!p) DEBUG_OUT.printf("%sRFRxPacket:%s packetbuffer getFront full\r\n",LON,LOFF);
      p->timestamp = micros(); // Micros does not increase in interrupt, but it can be used.
      p->packetsLost = lostPacketCount;
      radioDTU.setChannel(RxCH); //:::::::::::::::1
      uint8_t packetLen = radioDTU.getPayloadSize();
      if (packetLen > MAX_RF_PAYLOAD_SIZE)
        packetLen = MAX_RF_PAYLOAD_SIZE;
      radioDTU.read(p->packet, packetLen);

      packetBuffer.pushFront(p);
      if (!p) DEBUG_OUT.printf("%sRFRxPacket:%s packetbuffer pushFront full\r\n",LON,LOFF);
      else sts = true;
      lostPacketCount = 0;

      }
    else {
      // Buffer full. Increase lost packet counter.
      if (lostPacketCount < 255){
         lostPacketCount++;
         DEBUG_OUT.printf("%sRFRxPacket%s: lost packet, MI goes down !!\r\n",LON,LOFF);
         }
      DEBUG_OUT.printf("%sRFRxPacket%s: full buffer\r\n",LON,LOFF);
      sts = false;
      }
    radioDTU.flush_rx();// Flush buffer to drop the packet.

    radioDTU.startListening(); //:::::::::::::::2

  }//while
  isRxIrq = false;
  return sts;

}//----RFRxPacket-----------------------------------------------------------------------------------

#ifdef ESP8266
void ICACHE_RAM_ATTR RFirqHandler() {
#else
void RFirqHandler() {
#endif
//---------------------------------------------------------------------------------------

    DISABLE_EINT;
    radioDTU.maskIRQ(true, true, true); //disable irq if DISABLE_EINT doesnt work ;-)

    bool tx_ok, tx_fail, rx_ready;                // declare variables for IRQ masks
    radioDTU.whatHappened(tx_ok, tx_fail, rx_ready); // get values for IRQ masks
    // whatHappened() clears the IRQ masks also. This is required for
    // continued TX operations when a transmission fails.
    // clearing the IRQ masks resets the IRQ pin to its inactive state (HIGH)

    if (rx_ready){
        isRxIrq = true;
        //DEBUG_OUT.printf ("nrf rx ok %i\r\n", tx_ok);
        RFRxPacket();
        //DEBUG_OUT.printf ("nrf rx packet ok %i\r\n", tx_ok);
        }
    if (tx_ok) DEBUG_OUT.printf ("nrf Tx ok %i\r\n", tx_ok);
    //else DEBUG_OUT.printf ("nrf Tx ok ?? %i\r\n", tx_ok);
    if (tx_fail) DEBUG_OUT.printf ("nrf Tx fail %i\r\n", tx_fail);
    //else DEBUG_OUT.printf ("nrf Tx fail ?? %i\r\n", tx_fail);


//  if (tx_fail)            // if TX payload failed
//        radioDTU.flush_tx(); // clear all payloads from the TX FIFO

  radioDTU.maskIRQ(true, true, false);//Configuring IRQ pin to reflect data_ready events
  ENABLE_EINT;

}//---RFirqHandler-------------------------------------------------------------------

void   setRxPipe(void){
//---------------------------------------------------------------------------------------

  if (SNIFFER){
     radioDTU.openReadingPipe(0, 0x00aa);
     radioDTU.openReadingPipe(1, 0x0055);
     }
  else {
      //radioDTU.openReadingPipe(0, DTU_RADIO_ID);
      radioDTU.openReadingPipe(1, DTU_RADIO_ID);
      //radioDTU.openReadingPipe(1, WR1_RADIO_ID);
      }

}//----setRxPipe----------------------------------------------------------------------

static void RFConfig(void) {
//---------------------------------------------------------------------------------------

  while (!radioDTU.begin()) {
    DEBUG_OUT.printf("Radio hardware is not responding!!\r\n");
    delay(1000);
    }

  radioDTU.setAutoAck(0);
  radioDTU.setRetries(0, 0);

  radioDTU.setDataRate(DEFAULT_RF_DATARATE);
  radioDTU.disableCRC();
  // Use lo PA level, as a higher level will disturb CH340 DEBUG_OUT usb adapter
  radioDTU.setPALevel(PA_LEVEL);
  radioDTU.setPayloadSize(MAX_RF_PAYLOAD_SIZE);
  radioDTU.setAddressWidth(5);

  setRxPipe();
  // We want only RX irqs,
  if(INTERRUPT){
    // maskIRQ args = "data_sent", "data_fail", "data_ready"
    radioDTU.maskIRQ(true, true, false);//Configuring IRQ pin to reflect data_ready events
    //Configuring IRQ pin to reflect all events
    //radioDTU.maskIRQ(false, false, false);
    // disable IRQ masking for this step
    //radio.maskIRQ(true, true, true);

    //Attach interrupt handler to NRF IRQ output. Overwrites any earlier handler.
    attachInterrupt(digitalPinToInterrupt(RF1_IRQ_PIN), RFirqHandler, FALLING); // NRF24 Irq pin is active low.
    }
  // Initialize SerialHdr header's address member to promiscuous address.
  uint64_t addr = DTU_RADIO_ID;
  for (int8_t i = sizeof(SerialHdr.address) - 1; i >= 0; --i) {
    SerialHdr.address[i] = addr;
    addr >>= 8;
    }

  radioDTU.printDetails();
  delay(1000);
  UpdateTxMsgTick = millis() + 1000;
}//--RFConfig-------------------------------------------------------------------------------------


bool GetMIModel(void){  //probably we might need it more than once
//---------------------------------------------------------------------------------------
  uint64_t sn = SerialWR;
  longlongasbytes llsn;
  llsn.ull  = sn;

  DEBUG_OUT.printf("You defined inverter serial number as: %x%x%x!!!\r\n",llsn.bytes[5],llsn.bytes[4],llsn.ull);

  if(llsn.bytes[5] == 0x10) {
      switch(llsn.bytes[4]) {
        case 0x21:  WhichMI = MI300;  //MI300
             break;
        case 0x41:  WhichMI = MI600;  //MI600
             break;
        case 0x61:  WhichMI = MI1500;  //MI1500
             break;
        default:
             DEBUG_OUT.printf("Inverter model not known!!! check serial number\r\n");
             return false;
        }

      strcpy(MIWHAT,MItype[WhichMI].name);
      MIportPower = MItype[WhichMI].portP;
      if (NRofPV==0)
          NRofPV = MItype[WhichMI].NrPorts;  //if NRofPV not defined in settings.h, we set NRofPV here

      MAXPOWER = MItype[WhichMI].NrPorts * MIportPower; // rated power of inverter
      MINPOWER = int(MAXPOWER / 10); //below 10%, the inverter will be shut down
      DEBUG_OUT.printf("Inverter should be a %s, with MAXPOWER %iW, MINPOWER %iW\r\n",MIWHAT, MAXPOWER,MINPOWER);
      return true;
      }
  else{
      DEBUG_OUT.printf("Inverter model not known!!! check serial number\r\n");
      return false;
      }

}//---GetMIModel------------------------------------------------------------------------------------

void setup(void) {
//---------------------------------------------------------------------------------------
  XtimeB++;
  DEBUG_OUT.begin(SER_BAUDRATE);
  delay(5000);
  DEBUG_OUT.flush();
  DEBUG_OUT.printf(".....................\r\n");
  DEBUG_OUT.printf("Hoylmoly DTU for MI \r\n");
  DEBUG_OUT.printf(".....................\r\n");
  DEBUG_OUT.printf("Setup DTU,  wait....\r\n");

  while (! GetMIModel())
    delay (4000);

  if (CHECK_CRC)   DataOK = true;
  if (WR_LIMITTED) ZEROEXP = true;

  #ifdef ESP8266
    if (WITHWIFI){
      if(!SNIFFER){
          while (!setupWifi())
            DEBUG_OUT.printf("[WiFi] Setup, try again..\r\n");

          setupClock();
          DEBUG_OUT.printf("[MQTT] Setup .................\r\n");
          isMQTT = setupMQTT();
          DEBUG_OUT.printf("[HTTP] Setup .................\r\n");
          setupWebServer();
          #ifdef WITH_OTA
            setupUpdateByOTA();
          #endif
          calcSunUpDown (getNow());
          is_Day = isDayTime(TIMEOFFSET);
          //DEBUG_OUT.printf ("it is %s \r\n",(is_Day?"day time":"night time"));
          //hmPackets.SetUnixTimeStamp (getNow());
         }
      }
  #else
    //hmPackets.SetUnixTimeStamp(0x62456430);
  #endif
  delay(2000);
  //---NRF--------------------------
  DEBUG_OUT.printf("[NRF] Setup .................\r\n");
  // Configure nRF IRQ input
  if(INTERRUPT)
    pinMode(RF1_IRQ_PIN, INPUT);
  RFConfig();

  delay(1000);
  UpdateIPServicesTick = millis() + 200;
  if (WITHWIFI)
      STARTTIME=(String)getDateStr(getNow())+" "+(String)getTimeStr(getNow());

  DEBUG_OUT.printf("\r\n\r\nMicroinverter is %s%s with %i PV's%s, starting at ",LON,MIWHAT,NRofPV,LOFF);
  DEBUG_OUT.printf("\r\n%s",STARTTIME.c_str());
  TxLimitSts = false;

  DEBUG_OUT.printf("Hoylmoly Version %s \r\n",VERSION);
  DEBUG_OUT.printf("Setup finished ------------------------------------------------------\r\n\r\n");
  DEBUG_OUT.printf("%sType 1 and return for HELP%s\r\n\r\n",LON,LOFF);
  DEBUG_OUT.printf("%sif you do not receive anything, change PA_LEVEL first%s\r\n\r\n",LON,LOFF);
  SerCmd = 17 ;//get WR info first
}//---setup------------------------------------------------------------------------------------


static void RFTxPacket(uint64_t dest, uint8_t *buf, uint8_t len) {
//----------------------------------------------------------------------------------------------------

  radioDTU.flush_tx();

  if (DEBUG_TX_DATA) {     //packet buffer to output
    DEBUG_OUT.printf("RFTxPacket: CH%02i ",TxCH);
    for (uint8_t i = 0; i < len; i++){
      DEBUG_OUT.printf("%02X",buf[i]);
      }
    DEBUG_OUT.printf("\r\n");
  }
  //if(INTERRUPT) DISABLE_EINT;//?????
  radioDTU.stopListening(); //:::::::::::::::0
  radioDTU.setCRCLength(RF24_CRC_16);
  radioDTU.enableDynamicPayloads();
  radioDTU.setAutoAck(true);
  radioDTU.setRetries(3, 15); //5,15
  radioDTU.openWritingPipe(dest);
  radioDTU.setChannel(TxCH);
  //if(INTERRUPT) ENABLE_EINT;//?????
  uint8_t res = radioDTU.write(buf, len);
  //if (DEBUG_TX_DATA)
     //DEBUG_OUT.printf("..... res: %i\r\n", res);

   //radioDTU.print_status(radioDTU.get_status());

  // Try to avoid zero payload acks (has no effect)
  radioDTU.openWritingPipe(DUMMY_RADIO_ID);
  radioDTU.setAutoAck(false);
  radioDTU.setRetries(0, 0);
  radioDTU.disableDynamicPayloads();
  radioDTU.setCRCLength(RF24_CRC_DISABLED);

  radioDTU.setChannel(RxCH); //:::::::::::::::1
  radioDTU.startListening(); //:::::::::::::::2

}//----RFTxPacket-------------------------------------------------------------------------------------------------------


void HopCH(void){
//----------------------------------------------------------------------------------------------------
  static uint64_t LastTMO = 0;

  if ( (millis() - timeLastRxAck) > RxAckTimeOut){  //hop RxCH when timeout RXack
    if (RxAckTimeOut == TIMEOUTRXACK ) { //todo rxacktmo
      DEBUG_OUT.printf  ("%sHopCH%s: RxAck timeout RxCH%i millis:%lu lastAck:%lu  ",LON,LOFF, RxCH,millis(),timeLastRxAck);
      DEBUG_OUT.printf  ("RxAckTMO:%lu\r\n",RxAckTimeOut);
    }

    RxChId++;
    if (RxChId >= std::size(channels) )
      RxChId = 0;
    RxCH = channels[RxChId];
    timeLastRxAck = millis(); //reset it
    RxAckTimeOut = 150; //todo it was 100  , try faster to find out a channel
  }
  //tx allways hopping
  TxChId++;
  if (TxChId >= std::size(channels) )
    TxChId = 0;
  TxCH = channels[TxChId];

}//----HopCH----------------------------------------------------------------------------------------


void SerialCmdHandle(void){
//----------------------------------------------------------------------------------------------------------------------
    switch (SerCmd){
        case 1: //Output HELP, serial commands
          DEBUG_OUT.printf("\r\n\r\nSerial Commands:\r\n");
          DEBUG_OUT.printf("1:help\t\t\t2:Status\r\n");
          DEBUG_OUT.printf("3:PA_LOW\t\t4:PA_HIGH\t\t5:PA_MAX\r\n");
          DEBUG_OUT.printf("6:Sniffer\t\t7:ZeroEx\t\t8:OnlyRX\r\n");
          DEBUG_OUT.printf("9:ShowTX\t\t10:Wifi\t\t\t11:CRC\r\n");
          DEBUG_OUT.printf("12:reboot\t\t13:ShowRX\t\t14:IRQ\r\n");
          DEBUG_OUT.printf("15:LIMITabsORpr\t\t16:boot MI\t\t17:InverterInfo\r\n");
          DEBUG_OUT.printf("18:ShutdownInv\t\t19:\t\t20:\r\n");
          DEBUG_OUT.printf("22-29:ZexpFactor\t\t\t\t\r\n");
          DEBUG_OUT.printf("100-1999:limiting(W)\r\n\r\n\r\n"); //ToDo help
          SerCmd=0; //stop command
        break;
        case 2:
          DEBUG_OUT.printf("\r\n\r\nVersion\t %s started at ",VERSION);
          DEBUG_OUT.printf(STARTTIME.c_str()); //todo ,cant print (String) with printf??? yes we can use c_str()
          DEBUG_OUT.printf("DEBUG_RX_DATA \t%i\r\n",DEBUG_RX_DATA);
          DEBUG_OUT.printf("DEBUG_TX_DATA \t%i\r\n",DEBUG_TX_DATA);
          switch (PA_LEVEL){
              case 1:DEBUG_OUT.printf("PA_LEVEL_LOW \t%i\r\n",PA_LEVEL);
              break;
              case 2:DEBUG_OUT.printf("PA_LEVEL_HIGH \t%i\r\n",PA_LEVEL);
              break;
              case 3:DEBUG_OUT.printf("PA_LEVEL_MAX \t%i\r\n",PA_LEVEL);
              break;
              }
          DEBUG_OUT.printf("Boot seq \t%i\r\n",XtimeB);
          DEBUG_OUT.printf("WITHWIFI \t%i\r\n",WITHWIFI);
          DEBUG_OUT.printf("ZEROEXP \t%i\r\n",ZEROEXP);
          DEBUG_OUT.printf("INTERRUPT \t%i\r\n",INTERRUPT);
          DEBUG_OUT.printf("SNIFFER \t%i\r\n",SNIFFER);
          DEBUG_OUT.printf("ONLY_RX \t%i\r\n",ONLY_RX);
          DEBUG_OUT.printf("INTERRUPT \t%i\r\n",INTERRUPT);
          DEBUG_OUT.printf("CHECK_CRC \t%i\r\n",CHECK_CRC);
          DEBUG_OUT.printf("WITHMQTT \t%i\r\n",WITHMQTT);
          DEBUG_OUT.printf("TIMEOUTRXACK \t%i msek\r\n",RxAckTimeOut);
          DEBUG_OUT.printf("LIMITABSOLUT \t%i\r\n",LIMITABSOLUT);
          #ifdef WITH_OTA
            DEBUG_OUT.printf("WITH_OTA \t1\r\n");
          #endif
          DEBUG_OUT.printf("ZEXPFACTOR \t%i\r\n",FACTOR);
          static char buffer[30];
          IP2string (WiFi.localIP(), buffer);
          DEBUG_OUT.printf("IP\t\t%s\r\n",buffer);
          SerCmd=0; //stop command
        break;
        case 3:
           PA_LEVEL = RF24_PA_LOW;
           radioDTU.setPALevel(PA_LEVEL);
           DEBUG_OUT.printf("RF24_PA_LOW\r\n");
           radioDTU.printDetails();
           SerCmd=0; //stop command
        break;
        case 4:
          PA_LEVEL = RF24_PA_HIGH;
          radioDTU.setPALevel(PA_LEVEL);
          DEBUG_OUT.printf("RF24_PA_HIGH\r\n");
          radioDTU.printDetails();
          SerCmd=0; //stop command
        break;
        case 5:
           PA_LEVEL = RF24_PA_MAX;
           radioDTU.setPALevel(PA_LEVEL);
           DEBUG_OUT.printf("RF24_PA_MAX\r\n");
           radioDTU.printDetails();
           SerCmd=0; //stop command
        break;
        case 6:
              SNIFFER = (SNIFFER) ? 0 : 1;
              DEBUG_OUT.printf("CMD Sniffer %i\r\n", SNIFFER);
              RFConfig();
              SerCmd=0; //stop command
        break;
        case 7:
              ZEROEXP = (ZEROEXP) ? 0 : 1;
              DEBUG_OUT.printf("CMD Zeroexport %i\r\n",ZEROEXP);
              SerCmd=0; //stop command
        break;
        case 8:
              ONLY_RX = (ONLY_RX) ? 0 : 1;
              DEBUG_OUT.printf("CMD Only RX %i\r\n", ONLY_RX);
              SerCmd=0; //stop command
        break;
        case 9:
              DEBUG_TX_DATA = (DEBUG_TX_DATA) ? 0 : 1;
              DEBUG_OUT.printf("CMD Out TX Data %i\r\n",DEBUG_TX_DATA);
              SerCmd=0; //stop command
        break;
        case 10:
              WITHWIFI = (WITHWIFI) ? 0 : 1;
              //if (WITHWIFI)  WITHWIFI = 0;
              //else  WITHWIFI = 1;
              DEBUG_OUT.printf("CMD Wifi %i\r\n",WITHWIFI);
              SerCmd=0; //stop command
              setup();
        break;
        case 11:
              CHECK_CRC = (CHECK_CRC) ? 0 : 1;
              DEBUG_OUT.printf("CMD CHECK_CRC %i\r\n",CHECK_CRC);
              SerCmd=0; //stop command
        break;
        case 12: //reboot MI
              setup();
              SerCmd=0; //stop command
        break;
        case 13:
              DEBUG_RX_DATA = (DEBUG_RX_DATA) ? 0 : 1;
              DEBUG_OUT.printf("CMD DEBUG_RX_DATA %i\r\n",DEBUG_RX_DATA);
              SerCmd=0; //stop command
        break;
        case 14:
              INTERRUPT = (INTERRUPT) ? 0 : 1;
              DEBUG_OUT.printf("CMD INTERRUPT %i\r\n",INTERRUPT);
              setup();
              SerCmd=0; //stop command
        break;

        case 15:
              LIMITABSOLUT = (LIMITABSOLUT) ? 0 : 1;
              DEBUG_OUT.printf("CMD LIMITABSOLUT %i\r\n",LIMITABSOLUT);
              Limit = 0;
              TxLimitSts = false;
              SerCmd=0; //stop command
        break;
        case 16: //boot_mi cmd:0x51 (55AA)  will be send over RFisTime2Send to inverter
        break;
        case 17: //wrinfo  cmd:0x0f           will be send over RFisTime2Send to inverter
        break;
        case 18: //inverter shutdown cmd:0x51 (0xAA55)     will be send in RF RFisTime2Send to inverter
        break;
        case 21 ... 29: FACTOR = SerCmd - 20;
        case 30 ... 99: // this are empty, until now
        break;
        case 100 ... 1999:  //100-1999 limit watt will be send in RF RFisTime2Send over SerCmd
        break;
        default:
        break;
        }

}
//----SerialCmdHandle-----------------------------------------------------------------------------------------------------

void RFisTime2Send (void) {
//----------------------------------------------------------------------------------------------------------------------
  static uint8_t MIDataCMD = 0x36;   //begin with first PV
  static uint8_t MI600_DataCMD = 0x09 ;
  static uint8_t telegram = 0; //this is the  number of timesharing for tx commands to inverter
  int32_t size = 0;
  uint64_t dest = WR1_RADIO_ID;
  uint8_t UsrData[10];
  char Cmd = 0;
  int Limit2Tx = 0;

  if (millis() >= UpdateTxMsgTick) { //is time to tx commands to inverter
    UpdateTxMsgTick += TXTIMER;
    if (telegram > std::size(channels) )  telegram = 0; //reset telegram

    switch (WhichMI){ //choose which inverter
        case MI300: MIDataCMD=0x09; // (MI300) 1 PV
        break;
        case MI600: // (MI600)  2 PVs
            if (MI600_DataCMD == 0x09) MI600_DataCMD=0x11; //flipflop
            else if (MI600_DataCMD == 0x11) MI600_DataCMD=0x09;
            MIDataCMD=MI600_DataCMD;
        break;
        case MI1500:// (MI1500) // 4 PVs
            if (MIDataCMD > 0x0039) MIDataCMD= 0x0036;
        break;
        default:
            DEBUG_OUT.printf("%sRFisTime2Send:%s Wrong inverter type!!\r\n",LON,LOFF);
        break;
    }

    if (TxLimitSts) { //zeropower limiting
      Cmd=0x51;
      if (!isOnTx) DEBUG_OUT.printf("%sRFisTime2Send:%s CMD:%03X(0x5A5A) CH:%i set limit:%i\r\n",LON,LOFF,Cmd, TxCH,Limit);

      if (LIMITABSOLUT){  //set SubCmd and  UsrData for limiting
      // ---- todo check this with other MIs, my MI needs this after 500W, it seems to be not linear OR its a bug here ???
         if (Limit > 450) { Limit2Tx = 500 + ((Limit-500)*FACTOR); }
         else { Limit2Tx=Limit; }
      // ---- todo check this with other MIs, my MI needs it after 500W, it seems to be not linear OR its a bug here???

         UsrData[0]=0x5A;UsrData[1]=0x5A;UsrData[2]=0; //absolut watt limiting,UsrData[2] doesn't matter what
          //todo TxLimit hiByte/lowByte, is it ok?
         UsrData[3]=((Limit2Tx*10) >> 8) & 0xFF;   UsrData[4]= (Limit2Tx*10)  & 0xFF;   //WR needs 1 dec point= zB 100.3 W, :-)
         size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,5);
      }
      else {
        UsrData[0]=0x5A;UsrData[1]=0x5A;UsrData[2]= Limit; // % of rated power limiting
        size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,3);
      }
    isOnTx = true;
    }
    else
      if (SerCmd){ //if a serial command waiting,  after ack the command, SerCmd will be set to NULL
        switch (SerCmd){ //SrCmd, commands to Tx
           case 16: //boot wr  0x55AA
              Cmd=0x51;
              if (!isOnTx) DEBUG_OUT.printf("%sRFisTime2Send:%s CMD 0x%X(0x55AA) CH:%i Inverter boot request\r\n",LON,LOFF,Cmd,TxCH);
              UsrData[0]=0x55;UsrData[1]=0xAA;
              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,2);
           break;
           case 17: //request Inverter info
              Cmd=0x0f;
              UsrData[0]=0x01;
              if (!isOnTx) DEBUG_OUT.printf("%sRFisTime2Send:%s CMD 0x%X CH:%i  Sending InverterInfo request\r\n",LON,LOFF, Cmd, TxCH);
              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,1);
            break;
            case 18:
              Cmd=0x51; //todo request Inverter shutdown, but inverter does nothing, still producing???
              if (!isOnTx) DEBUG_OUT.printf("%sRFisTime2Send:%s CMD 0x%X(0xAA55) CH:%i Inverter shut down request\r\n",LON,LOFF, Cmd, TxCH);
              UsrData[0]=0xAA;UsrData[1]=0x55;
              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,2);
            break;
            case 21://WR_HW_SW  doesnt work              Cmd=0x06;
            case 100 ... 1999: //Limiting over serial command
                 TxLimitSts = true;
                 DEBUG_OUT.printf("%sRFisTime2Send:%s CMD 0x%X CH:%i set limiting over serial command\r\n",LON,LOFF, Cmd, TxCH);
            break;
        }
        isOnTx = true; // we are sending, not need to show it continuously
      }
      else {//no SerCmd or Limiting,  usual operation : request WR data
        TxLimitSts = false;
        UsrData[0]=0x0;//set SubCmd and  UsrData for data request
        size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, MIDataCMD, UsrData,1);
      }
    RFTxPacket(dest, (uint8_t *)&sendBuf, size);
    telegram++;
    if (WhichMI == MI1500)  MIDataCMD++;  //next PV, if 4PV modell

  } //if millis
}//----RFisTime2Send---------------------------------------------------------------------------------------

void RFDumpRxPacket(NRF24_packet_t *p, uint8_t payloadLen) {
//------------------------------------------------------------------------------------------------------
  DEBUG_OUT.printf("CH %i ",RxCH);
  // Write  packets lost, address and payload length
  dumpData((uint8_t *)&SerialHdr.packetsLost, sizeof(SerialHdr.packetsLost));
  dumpData((uint8_t *)&SerialHdr.address, sizeof(SerialHdr.address));
  // Trailing bit?!?
  dumpData(&p->packet[0], 2);
  // Payload length from PCF
  dumpData(&payloadLen, sizeof(payloadLen));
  // Packet control field - PID Packet identification
  uint8_t val = (p->packet[1] >> 1) & 0x03;
  DEBUG_OUT.printf("%i ",val);
  if (payloadLen > 9) {
    dumpData(&p->packet[2], 1);
    dumpData(&p->packet[3], 4);
    dumpData(&p->packet[7], 4);

    uint16_t remain = payloadLen - 2 - 1 - 4 - 4 + 4;

    if (remain < 32) {
      dumpData(&p->packet[11], remain);
      DEBUG_OUT.printf("%04X ", crc);
      if (((crc >> 8) != p->packet[payloadLen + 2]) || ((crc & 0xFF) != p->packet[payloadLen + 3]))
        DEBUG_OUT.printf("%i",0);
      else
        DEBUG_OUT.printf("%i",1);
    }
    else DEBUG_OUT.printf("remain %i ",remain);
  }
  else {
    dumpData(&p->packet[2], payloadLen + 2);
    DEBUG_OUT.printf("%04X ", crc);
  }
  DEBUG_OUT.printf("\r\n",NULL);
}//----RFDumpRxPacket-------------------------------------------------------------------------------

uint8_t PVcheck(bool reset=false){
//-------------------------------------------------------------------------------------------------

  if ( (millis() - timeCheckPV) > TIMERPVCHECK){  //actualize every TIMERPVCHECK msek
     timeCheckPV = millis();
     pvCnt[0]=pvCnt[1]=pvCnt[2]=pvCnt[3]=0; //reset PV sts
     DEBUG_OUT.printf("%sPVcheck:%s timer %i\r\n",LON,LOFF,timeCheckPV);
     return 0;
  }

  switch (NRofPV){
    case 1: if (pvCnt[0]==NRofPV) return NRofPV;
    break;
    case 2: if ((pvCnt[0]+pvCnt[1])==NRofPV) return NRofPV;
    break;
    case 3: if ((pvCnt[0]+pvCnt[1]+pvCnt[2])==NRofPV) return NRofPV;
    break;
    case 4: if ((pvCnt[0]+pvCnt[1]+pvCnt[2]+pvCnt[3])==NRofPV) return NRofPV;
    break;
  }
  return 0;
}//----------------------------------------------------------------------------------------------

void SerialRxHandle(void){ //read from serial for WR control cmd's
//-------------------------------------------------------------------------------------------
  static uint8_t InCnt=0;
  int temporary=0;

  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    if (incomingByte != 13){ //CR
      SerialIn[InCnt]=incomingByte;
      InCnt++;
    }
    else {
      SerialIn[InCnt]=0;   //eofl
      temporary = atoi(SerialIn);
      SerCmd = temporary;
      DEBUG_OUT.printf("%sSerialIn:%s %s int:%i SerCmd:%i\r\n",LON,LOFF,SerialIn,temporary,SerCmd);

      if ((temporary >1999) || (temporary < 100)){  //other cmds. implemented, request WR info etc.
        SerCmd = temporary;      //this is a serial command
        TxLimitSts = false;      //no Limit send needed
        SerialCmdHandle();
        }
      else{ //this is a limiting command 100 to 1999 as watt
        Limit = temporary; //Limit is 100-1999 watt  here

        if (!LIMITABSOLUT)
           Limit = round(Limit *100 / MAXPOWER); //Limit is % of rated power now
        SerCmd = temporary; //this is a Limiting command, will be handled in RFisTime2Send()
        DEBUG_OUT.printf("%sSerialIn:%s %s SerCmd:%i Limit:%i\r\n",LON,LOFF,SerialIn,SerCmd,Limit);
      }
      InCnt=0;
    }
  }
}//---SerialRxHandle-------------------------------------------------------------------------------

void SendMQTTMsg(String topic, String value){
//-------------------------------------------------------------------------------------------
  if (WITHWIFI && isMQTT){
    if (!checkWifi()) return;
    mqttClient.beginMessage(topic);
    mqttClient.print(value);
    mqttClient.endMessage();
  }

}//---SendMQTTMsg-------------------------------------------------------------------------------------------------------

void SendMQTTJSON(String topic) {
//------------------------------------------------------------------------------------------------------------
      // compose and sent JSON Message
  String json = String("{\"TIME\":\"")+(String)getDateStr(getNow())+String("T")+(String)getTimeStr(getNow())+String("\", \"ENERGY\":{")+
  String("\"Power\":")+String(PMI)+String(", ")+
  String("\"Limit\":")+String(Limit)+String(", ")+
  String("\"Power_"+(String)(PV+1)+"\":")+String(P_DC)+String(", ")+
  String("\"Voltage_"+(String)(PV+1)+"\":")+String(U_DC)+String(", ")+
  String("\"Current_"+(String)(PV+1)+"\":")+String(I_DC)+String(", ")+
  String("\"Energy_"+(String)(PV+1)+"\":")+String(Q_DC)+String(", ")+
  String("\"Temperature\":")+String(TEMP)+String(", ")+
  String("\"Status"+(String)(PV+1)+"\":\"")+String(STAT)+String("\"")+
  String("}}");

  mqttClient.beginMessage(topic);
  mqttClient.print(json);
  mqttClient.endMessage();
}//----SendMQTTJSON-----------------------------------------------------------------------------------------------------

void SendMQTTTopics (void){
//----------------------------------------------------------------------------------------------------------------------
  SendMQTTMsg((String)INVTOT_P, (String) PMI);
  SendMQTTMsg((String)LIMIT_P, (String) Limit);
  SendMQTTMsg((String)INV_P_PVNR+(String)(PV+1), (String) P_DC);
  SendMQTTMsg((String)INV_UDC_PVNR+(String)(PV+1), (String) U_DC);
  SendMQTTMsg((String)INV_IDC_PVNR+(String)(PV+1), (String) I_DC);
  SendMQTTMsg((String)INV_Q_PVNR+(String)(PV+1), (String) Q_DC);
  SendMQTTMsg((String)INV_TEMP, (String) TEMP);
  SendMQTTMsg((String)INV_STS_PVNR+(String)(PV+1), (String) STAT);
  SendMQTTMsg((String)INV_CONSUM_P, (String) (abs(GridPower) + PMI));
  SendMQTTMsg((String)DAY, (String)getDateStr(getNow()));
  SendMQTTMsg((String)TIME, (String)getTimeStr(getNow()));
  SendMQTTMsg((String)INFO, "NRF24");
}//----SendMQTTTopics--------------------------------------------------------------------------------------------------

void PrintOutValues(void){
//----------------------------------------------------------------------------------------------------------------------
  DEBUG_OUT.printf("CH:%02i %04iW [PV%1i %5sW %4sV %4sA %04iWh][%5sV %4sHz %4sC S:%i] Grid:%04iW Lmt:%04i%s PVok:%i  ",
  RxCH,
  (int)PMI,
  PV,
  String(P_DC,1),
  String(U_DC,1),
  String(I_DC,1),
  (int)Q_DC,
  String(U_AC,1),
  String(F_AC,1),
  String(TEMP,1),
  STAT,
  (int)GridPower,
  Limit,((LIMITABSOLUT)?"W":"%"),
  PVcheck());

  uint64_t t=millis(); //print time since started
  uint16_t s = (uint16_t) (t / 1000) % 60;
  uint16_t m = (uint16_t) ((t / (1000 * 60)) % 60);
  uint16_t h = (uint16_t) ((t / (1000 * 60 * 60)) % 24);
  uint16_t d = (uint16_t) (t / (1000 * 60 * 60 * 24));
  uint16_t ms = (uint16_t) (t % 1000);

  DEBUG_OUT.printf("%02i:%02i:%02i:%02i:%02i\r\n",d,h,m,s,ms );

  }
//---PrintOutValues-----------------------------------------------------------------------------------------------------

void MI1500DataMsg(NRF24_packet_t *p){
//--------------------------------------------------------------------------------------------------
  U_DC =  (float) ((p->packet[11] << 8) + p->packet[12])/10;
  I_DC =  (float) ((p->packet[13] << 8) + p->packet[14])/10;
  U_AC =  (float) ((p->packet[15] << 8) + p->packet[16])/10;
  F_AC =  (float) ((p->packet[17] << 8) + p->packet[18])/100;
  P_DC =  (float)((p->packet[19] << 8) + p->packet[20])/10;
  Q_DC =  (float)((p->packet[21] << 8) + p->packet[22])/1;
  TEMP =  (float) ((p->packet[23] << 8) + p->packet[24])/10;

  if (!CHECK_CRC)
    if ((30<U_DC<50) && (0<I_DC<15) && (200<U_AC<300) && (45<F_AC<55) && (0<P_DC<420) && (0<TEMP<80))
      DataOK = true;  //we need to check this, if no crc
    else { DEBUG_OUT.printf("%sMIDataMsg:%s Wrong data!!\r\n",LON,LOFF);DataOK = false; return;}

  STAT = (uint8_t)(p->packet[25] );
  FCNT = (uint8_t)(p->packet[26]);
  FCODE = (uint8_t)(p->packet[27]);

  if (p->packet[2] == 0xB6)  {PV= 0; TotalP[1]= P_DC; pvCnt[0]= 1;}//port 1
  if (p->packet[2] == 0xB7)  {PV= 1; TotalP[2]= P_DC; pvCnt[1]= 1;}//port 2
  if (p->packet[2] == 0xB8)  {PV= 2; TotalP[3]= P_DC; pvCnt[2]= 1;}//port 3
  if (p->packet[2] == 0xB9)  {PV= 3; TotalP[4]= P_DC; pvCnt[3]= 1;}//port 4
  TotalP[0]=TotalP[1]+TotalP[2]+TotalP[3]+TotalP[4];//in TotalP[0] is the totalPvW
  if((P_DC>MIportPower) || (P_DC<0) || (TotalP[0]>MAXPOWER)){// cant be!!
    DEBUG_OUT.printf("%sMI1500DataMsg:%s Wrong Data.. PV%1i %5sW Total:%5sW \r\n",LON,LOFF, PV,String(P_DC,1),String(TotalP[0],1) );
    TotalP[0]=0;
    return;
  }
#ifdef ESP8266
  VALUES[PV][0]=PV;
  VALUES[PV][1]=P_DC;
  VALUES[PV][2]=U_DC;
  VALUES[PV][3]=I_DC;
  VALUES[PV][4]=Q_DC;
  VALUES[PV][5]=STAT;
  VALUES[PV][6]=FCNT;
  VALUES[PV][7]=FCODE;
#endif
  PMI=TotalP[0];
  LIM=(uint16_t)Limit;
  PrintOutValues();
  //sprintf (cStr," %s", (String)getTimeStr(getNow()));

}//--MI1500DataMsg------------------------------------------------------------------------------------------------------

void MI600StsMsg (NRF24_packet_t *p){
  //-----------------------------------------------------------------------
  STAT = (int)((p->packet[11] << 8) + p->packet[12]);
  FCNT = (int)((p->packet[13] << 8) + p->packet[14]);
  FCODE = (int)((p->packet[15] << 8) + p->packet[16]);
#ifdef ESP8266
  VALUES[PV][5]=STAT;
  VALUES[PV][6]=FCNT;
  VALUES[PV][7]=FCODE;
#endif
}//--MI600StsMsg--------------------------------------------------------------------------------------------------------

void MI600DataMsg(NRF24_packet_t *p){
  //--------------------------------------------------------------------------------------------------------------------
  U_DC =  (float) ((p->packet[11] << 8) + p->packet[12])/10;
  I_DC =  (float) ((p->packet[13] << 8) + p->packet[14])/10;
  U_AC =  (float) ((p->packet[15] << 8) + p->packet[16])/10;
  F_AC =  (float) ((p->packet[17] << 8) + p->packet[18])/100;
  P_DC =  (float)((p->packet[19] << 8) + p->packet[20])/10;
  Q_DC =  (float)((p->packet[21] << 8) + p->packet[22])/1;
  TEMP =  (float) ((p->packet[23] << 8) + p->packet[24])/10;

  if (!CHECK_CRC)
    if ((30<U_DC<50) && (0<I_DC<15) && (200<U_AC<300) && (45<F_AC<55) && (0<P_DC<420) && (0<TEMP<80))
      DataOK = true;  //we need to check this, if no crc
    else {
      DEBUG_OUT.printf("%sMIDataMsg:%s Wrong data!!\r\n",LON,LOFF);DataOK = false;
      return;
    }

  if (p->packet[2] == 0x89)  {PV= 0; TotalP[1]= P_DC; pvCnt[0]= 1;}//port 1
  if (p->packet[2] == 0x91)  {PV= 1; TotalP[2]= P_DC; pvCnt[1]= 1;}//port 2

  TotalP[0]=TotalP[1]+TotalP[2]+TotalP[3]+TotalP[4];//in TotalP[0] is the totalPV power
  if((P_DC>400) || (P_DC<0) || (TotalP[0]>MAXPOWER)){// cant be!!
    TotalP[0]=0;
    return;
  }
#ifdef ESP8266
  VALUES[PV][0]=PV;
  VALUES[PV][1]=P_DC;
  VALUES[PV][2]=U_DC;
  VALUES[PV][3]=I_DC;
  VALUES[PV][4]=Q_DC;
#endif
  PMI=TotalP[0];
  LIM=(uint16_t)Limit;
  PrintOutValues();
}//--------------------------------------------------------------------------------------------------

void MIAnalysePacket(NRF24_packet_t *p,uint8_t payloadLen){
//--------------------------------------------------------------------------------------------------
  static bool ackMIinfo[3] = {false,false,false} ;
  int i=0;
  String a ="";
  switch (p->packet[2])  {
    case 0xD1: //ACK from CMD:0x51 Limiting, boot MI,..
      TxLimitSts = false;//stop sending 0x51 command
      if ( (p->packet[11] == 0x5A) && (p->packet[12] == 0x5A)) { //Power Limiting
        DEBUG_OUT.printf("%sMIAnalysePacket:%s ACK PowerLimiting(0x5A5A), CMD:%X RxCH:%i\r\n",LON,LOFF, p->packet[2],RxCH);
        pvCnt[0]=pvCnt[1]=pvCnt[2]=pvCnt[3]=0; //reset PV;sts
      }
      if ( (p->packet[11] == 0x55) && (p->packet[12] == 0xAA)) { //boot inverter
        DEBUG_OUT.printf("%sMIAnalysePacket:%s ACK boot inverter(0x55AA), CMD:%X RxCH:%i\r\n",LON,LOFF, p->packet[2],RxCH);
      }
      if ( (p->packet[11] == 0xAA) && (p->packet[12] == 0x55)) { //shutdown inverter
        DEBUG_OUT.printf("%sMIAnalysePacket:%s ACK shutdown inverter(0xAA55), CMD:%X RxCH:%i\r\n",LON,LOFF, p->packet[2],RxCH);
      }
      SerCmd = 0; //ack is here, stop sending
      timeLastRxAck = millis();
      RxAckTimeOut = TIMEOUTRXACK;
      isOnTx = false;
    break;
//    case 0x82:0x82 Gongfa also doesnt work
//    break;
//    case 0x86: //RF SW HW  ? DOESNT work on MI's!!
//    break;
    case 0x8f: //ACK InverterInfo from CMD:0x0f
      DEBUG_OUT.printf("%sMIAnalysePacket:%s ACK InverterInfo(0x0F),CMD:%x RxCH:%i\r\n",LON,LOFF,p->packet[2],RxCH);
      DEBUG_OUT.printf("\r\n%sMI %x:%x:%x:%x ",LON,p->packet[7],p->packet[8],p->packet[9],p->packet[10]);

      switch (p->packet[11]){  //RX from inverter
        case 0:// Command Receipt - First Frame
          DEBUG_OUT.printf("USFWBLD:%i.%i ",p->packet[12],p->packet[13]);
          DEBUG_OUT.printf("APFWBLD:%i.%i ",p->packet[14],p->packet[15]);
          DEBUG_OUT.printf("APPFDate:%i-",(p->packet[16]<<8) + p->packet[17]); //(YYYYd-)
          i = ((p->packet[18]<<8) + p->packet[19]);
          a= String( int(i /100))+"-"+String( int(i % 100));
          DEBUG_OUT.printf("%s ",a); //MM-DD
          i = ((p->packet[20]<<8) + p->packet[21]);
          a= String( int(i /100))+":"+String( int(i % 100));
          DEBUG_OUT.printf(" %s%s",a,LOFF); //hh:mm
          ackMIinfo [0]= true;
        break;
        case 1:// Command Receipt - second Frame
          DEBUG_OUT.printf("HWPN:%x.%x.%x.%x ",p->packet[12],p->packet[13],p->packet[14],p->packet[15]);
          DEBUG_OUT.printf("HWFBTLmValue:%x.%x ",p->packet[16],p->packet[17]);
          DEBUG_OUT.printf("HWFBReSPRT:%x.%x ",p->packet[18],p->packet[19]);
          DEBUG_OUT.printf("GridSamp:%x.%x ",p->packet[20],p->packet[21]);
          DEBUG_OUT.printf("ECapValue:%x.%x ",p->packet[22],p->packet[23],LOFF);
          DEBUG_OUT.printf(" MatchAPPFWPN:%x.%x.%x.%x%s",p->packet[24],p->packet[25],p->packet[26],p->packet[27],LOFF);//HHMM
          ackMIinfo [1]= true;
        break;
        case 2:// Command Receipt - third Frame
          DEBUG_OUT.printf("APPFW_MINVER:%x.%x ",p->packet[12],p->packet[13]);
          DEBUG_OUT.printf("HWINFOadr:%x.%x ",p->packet[14],p->packet[15]);
          DEBUG_OUT.printf("PNInfoCRC:%x.%x %s",p->packet[16],p->packet[17],LOFF);
          ackMIinfo [2]= true;
        break;
        default:
          DEBUG_OUT.printf(" this frame is not known:%i",p->packet[11]);
      }
      DEBUG_OUT.printf(" frame:%i %s\r\n\r\n",p->packet[11],LOFF);
      //todo      if ( ackMIinfo[0] && ackMIinfo[1] && (ackMIinfo[2]) ){ //never got the frame 3, why????
          SerCmd = 0; //ack is here, stop sending CMD: InverterInfo after all 3 frames are received
      //    ackMIinfo[0] = ackMIinfo[1] = ackMIinfo[2]= false;
      //   }
      timeLastRxAck = millis(); RxAckTimeOut = TIMEOUTRXACK;
      isOnTx = false;
    break;

    case 0xB6:    //4 ports
    case 0xB7:    //4 ports
    case 0xB8:    //4 ports
    case 0xB9:    //4 ports
      timeLastRxAck = millis(); RxAckTimeOut = TIMEOUTRXACK;
      MI1500DataMsg(p);
    break;

    case 0x89:    //1-2 ports
    case 0x91:    //2 ports
      timeLastRxAck = millis(); RxAckTimeOut = TIMEOUTRXACK;
      MI600DataMsg(p);
    break;

    case 0x88:    //1-2 ports
    case 0x92:    //2 ports
      timeLastRxAck = millis(); RxAckTimeOut = TIMEOUTRXACK;
      MI600StsMsg(p);
    break;
    default:
       DEBUG_OUT.printf("%sMIAnalysePacket:%s new CMD  %x \t",LON,LOFF, p->packet[2]);
       RFDumpRxPacket (p, payloadLen); //output received data
    }

}//--MIAnalysePacket----------------------------------------------------------------------------------

void RFRxAnalyse(void) {
//--------------------------------------------------------------------------------------------------
  while (!packetBuffer.empty()) {
    timeLastPacket = millis();
    // One or more records present
    NRF24_packet_t *p = packetBuffer.getBack();
    // Shift payload data due to 9-bit packet control field
    for (int16_t j = sizeof(p->packet) - 1; j >= 0; j--) {
     if (j > 0)
        p->packet[j] = (byte)(p->packet[j] >> 7) | (byte)(p->packet[j - 1] << 1);
     else
        p->packet[j] = (byte)(p->packet[j] >> 7);
     }

    SerialHdr.timestamp   = p->timestamp;
    SerialHdr.packetsLost = p->packetsLost;
    // Check CRC
    crc = 0xFFFF;
    crc = crc16((uint8_t *)&SerialHdr.address, sizeof(SerialHdr.address), crc, 0, BYTES_TO_BITS(sizeof(SerialHdr.address)));
    // Payload length
    uint8_t payloadLen = ((p->packet[0] & 0x01) << 5) | (p->packet[1] >> 3);
    // Add one byte and one bit for 9-bit packet control field
    crc = crc16((uint8_t *)&p->packet[0], sizeof(p->packet), crc, 7, BYTES_TO_BITS(payloadLen + 1) + 1);

    if ( (DEBUG_RX_DATA) || (SNIFFER) )
      RFDumpRxPacket (p, payloadLen); //output received data

    if (CHECK_CRC) {
      // If CRC is invalid only show lost packets
      if (((crc >> 8) != p->packet[payloadLen + 2]) || ((crc & 0xFF) != p->packet[payloadLen + 3])) {
        if (p->packetsLost > 0) {
          DEBUG_OUT.printf("%sRFRxAnalyse:%s CRC lost packets: %i",LON,LOFF,p->packetsLost);
        }
        packetBuffer.popBack();
        continue;
      }
      // Dump a decoded packet only once
      if (lastCRC == crc) {
        packetBuffer.popBack();
        continue;
      }
      lastCRC = crc;
    }// if checkcrc
    // Don't dump mysterious ack packages
    if (payloadLen == 0) {
      packetBuffer.popBack();
      DEBUG_OUT.printf("%sRFRxAnalyse:%s mysterious ack \r\n",LON,LOFF);
      continue;
    }

    if (p->packetsLost > 0) {
      DEBUG_OUT.printf("%sRFRxAnalyse:%s packet lost: %i\r\n",LON,LOFF, p->packetsLost);
      continue;
    }
    if (!SNIFFER)
      MIAnalysePacket(p,payloadLen);

    packetBuffer.popBack(); // Remove record as we're done with it.
  } //while
//  packetBuffer.popBack();??????????????
}//-----RFRxAnalyse-------------------------------------------------------------------------------


void DoZeroExport(void){
//-------------------------------------------------------------------------------------------------
/* Be aware!
   DTSU Export power must be PLUS
   DTSU Import power must be MINUS
*/
  int OverP;

  if (millis() < UpdateZeroExpTick){//wait for zeroexport timer
    DEBUG_OUT.printf("%sZeroExport:%s timer, not yet !\r\n",LON,LOFF);
    return; //not in first 60 sek
  }
  else if (WR_LIMITTED) UpdateZeroExpTick += FIXLIMITTICK;//if fixed limitation, set timer a bit longer
       else UpdateZeroExpTick += ZEXPUPDATETICK;

  if (!SmartMeterOk){
    DEBUG_OUT.printf("%sZeroExport:%s Smartmeter is not ready!\r\n",LON,LOFF);
    return;
  }

  if (WR_LIMITTED){  //inverter is permanently limited, producing fixed power
    Limit = WR_LIMITTED;
    GridPower = TOLERANCE+1; //;-)
    DEBUG_OUT.printf("%sZeroExport:%s fixed limiting inverter to %i\r\n",LON,LOFF,Limit);
    TxLimitSts = true;
    return;
  }
  if ((!TxLimitSts) && SmartMeterOk){  //we allow to send and smartmeter is alive
    if (abs (GridPower) > TOLERANCE) { // if it changes more than TOLERANCE watt
      if (GridPower >0){ //Export P is PLUS on DTSU666, MI producing too much power, need less power
          Limit= PMI - abs(GridPower);  //327-abs(296)
      }
      else { //Import P is MINUS on DTSU666, MI producing too low power, needing more power
          Limit= PMI+ abs(GridPower); //327+abs(-296)
      }

      if ((HH >= AFTERHH)&&(Limit <= MINPOWER)) {
        Limit = MINPOWER+20; //watt, do not shut down inverter after 17h
        DEBUG_OUT.printf("%sZeroExport:%s after %ih limit is %i \r\n",LON,LOFF,AFTERHH,(int)Limit);
        }
      // ToDo  -----zero export fine tuning--------------------------------------------------------
      //     (GridPower) ? (OverP = PMI - abs(GridPower)):(OverP= PMI + abs(GridPower)); below better readable
      if ( GridPower < 0) { OverP= PMI + abs(GridPower); }
      else { OverP = PMI - abs(GridPower); }

      if ( (MINPOWER >= OverP) && (OverP >= (int)(MINPOWER/2)) ) {
          Limit = MINPOWER+10; //do not shutdown inverter when more than half minP is used
          DEBUG_OUT.printf("%sZeroExport:%s OverP:%i > MINP/2:%i, Limit is min.: %i \r\n",LON,LOFF, (int)OverP,(int)(MINPOWER/2),(int)Limit);
        }
      //else DEBUG_OUT.printf("%sZeroExport:%s not doing this (OverP:%i <  MINP/2:%i), Limit: %i \r\n",LON,LOFF, (int)OverP,(int)(MINPOWER/2),(int)Limit);
      // ToDo  -----zero export fine tuning--------------------------------------------------------

      if (Limit >= MAXPOWER) Limit = MAXPOWER; //Limit is still in watt here

      if (!LIMITABSOLUT)  //if we want to have the percent % of rated power limiting
          Limit = round(Limit *100 / MAXPOWER); //Limit is now % of rated power

      if (!TxLimitSts) TxLimitSts = true; //we can send
      DEBUG_OUT.printf("%sZeroExport:%s GridPower out of tolerance %i W, Limiting %i\r\n",LON,LOFF, (int)GridPower,(int)Limit);
    }
    else {
      TxLimitSts = false;
      DEBUG_OUT.printf("%sZeroExport:%s GridPower inside tolerance %i W, do nothing\r\n",LON,LOFF, (int)GridPower);
    }
    SmartMeterOk = false; //wait again on new data
  }
}//---DoZeroExport------------------------------------------------------------------------------------------------------

void loop(void) {
//======================================================================================================================

  HopCH(); //Rx/Tx channel hopping, Rx with a timer timeout

  radioDTU.stopListening();
  radioDTU.setChannel(RxCH);
  radioDTU.startListening();

  if (!INTERRUPT)//RF polling if NOT INTERRUPT  defined
    RFRxPacket();

  delay(10); //this is very important !! without it, we were too fast in loop!!

  RFRxAnalyse();//analyse RF packet if any received

  if (! SNIFFER){
    SerialRxHandle();  //read from serial console any command
    if ((!ONLY_RX) && (is_Day)) // sending packet only in day time
        RFisTime2Send();//send packet over NRF
  }
  PVcheck(); //do we got all PV's data?
  #ifdef ESP8266
    if (WITHWIFI && (!SNIFFER)){
       if (!checkWifi()){
         setup();
       }
       if (millis() >= UpdateIPServicesTick){//not overload the web&mqtt server
         is_Day = isDayTime(TIMEOFFSET);
         if (!is_Day) {//at night, the website should not show old data
            for (byte pv=0; pv < NRofPV; pv++)
              for (byte i = 0; i < (ANZAHL_VALUES); i++)
                VALUES[pv][i]=0;
         }
         UpdateIPServicesTick += IPServicesUPDATETICK;

         webserverHandle(); //we publishing what we have

         if (isMQTT && WITHMQTT)
            mqttClient.poll();
         else if (WITHMQTT)
                setupMQTT();

         #ifdef WITH_OTA
            checkUpdateByOTA();
         #endif

         if (PVcheck()){
             if (ZEROEXP)
                DoZeroExport();

             if (isMQTT && DataOK ){  //send mqtt if only inverter data ok
               mqttClient.poll();
               #ifdef SENDJSON
                   SendMQTTJSON(JSON_TOPIC);
               #else
                   SendMQTTTopics();
               #endif
             }
             else {
               if (!isMQTT && WITHMQTT) isMQTT=setupMQTT(); //mqtt was not connected or lost connection
             }
         }//PVcheck
       }//update
    }//wifi
  #endif //esp8266

}//-----loop-----------------------------------------------------------------------------------------
// todo
// todo wr status
/*
PORT_STATUS =
["no data?",
"?",
"?gesehen",
"Normal", (3)
"?",
"MPPT port not connected", (5)
"?gesehen",
"?",
"Reduced"] (8)
*/