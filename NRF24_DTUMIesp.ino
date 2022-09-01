/* die orig. SW ist vom Hubi, wurde von mir(Ziyat T.) für den MI-WR abgeaendert.
Getestet auf ESP8266/ArduinoUNO.
https://www.mikrocontroller.net/topic/525778
https://github.com/hm-soft/Hoymiles-DTU-Simulation
----------------------------------------
Alle Einstellungen sind in Settings.h UND secrets.h !!
----------------------------------------
*/
#define VERSION "V0.1.6"


#include <stdint.h>
//#include <printf.h>

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <RF24_config.h>
#include "printf.h"
#include "Settings.h"
#include "CircularBuffer.h"
#include "hm_crc.h"
#include "hm_packets.h"
#include "Debug.h"
#include "Sonne.h"

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
static uint64_t       UpdateMqttTick=0;
static uint64_t       UpdateZeroExpTick=0;

static uint64_t RxAckTimeOut = 100;   // at begin, milli sek. wenn zu lange nichts kommt, müssen wir wechseln;
static uint64_t timeLastPacket = millis();
static uint64_t timeLastAck    = RxAckTimeOut*4; //at the start
static uint64_t timeCheckPV = 100;
// Set up nRF24L01 radio on SPI bus plus CE/CS pins
// If more than one RF24 unit is used the another CS pin than 10 must be used
// This pin is used hard coded in SPI library

static RF24 radio1 (RF1_CE_PIN, RF1_CS_PIN);
static NRF24_packet_t bufferData[PACKET_BUFFER_SIZE];
static CircularBuffer<NRF24_packet_t> packetBuffer(bufferData, sizeof(bufferData) / sizeof(bufferData[0]));
static Serial_header_t SerialHdr;


static uint16_t lastCRC;
static uint16_t crc;

static uint8_t channels[] = {3,23, 40, 61, 75};   //{1, 3, 6, 9, 11, 23, 40, 61, 75}
static uint8_t TxChId = 0;
static uint8_t RxChId = 0;                         // fange mit 3 an
static uint8_t TxCH   = channels[TxChId];
static uint8_t RxCH   = channels[RxChId];


#ifdef ESP8266
    char * getChannelName (uint8_t i);
    static const int    ANZAHL_VALUES         = 8;
    static float        VALUES[4][ANZAHL_VALUES] = {};
    static const char   *CHANNEL_NAMES[ANZAHL_VALUES]
       = {"PanelNr   ",
          "P [W]  ",
          "Udc [V]",
          "Idc [A]",
          "E [Wh] ",
          "Status ",
          "FCnt   ",
          "FCode  "};
#endif
//static const uint8_t DIVISOR[ANZAHL_VALUES] = {1,1,1,1,1,1,1,1,1,1,1,1};
static const char BLANK = ' ';


static bool isIrq = false;
static char CHANNELNAME_BUFFER[15]="";


static int SerCmd=0;
static char cStr[100];
//char WRdata[120];
static float  U_DC =0;
static float  I_DC =0;
static float  U_AC =0;
static float  F_AC =0;
static float  P_DC =0;
static float  Q_DC =0;
static float  TEMP =0;

static int  STAT =0;
static int  FCNT = 0;
static int  FCODE = 0;
static uint8_t  PV = 0;
static uint16_t PMI = 0;
//uint16_t P_DSU = 0;
static uint16_t LIM=0; //for ModWebserver
static uint8_t DataOK=0;
static uint8_t pvCnt[4]={0,0,0,0};


static float TotalP[5]={0,0,0,0,0}; //0 is total power, 1-4 are 4 PV power

#ifdef ESP8266
  #include "ModWebserver.h"
  //#include "Sonne.h"
#endif

//MI-WR Data
uint8_t sendBuf[MAX_RF_PAYLOAD_SIZE];


//read from Serial for testing
static char SerialIn[10]="";

static uint8_t WRInfo=1;

//String mStr = "";     // empty string

static bool TxLimitSts = false; //quiet
static uint16_t Limit=0; //setup
//static uint16_t OldLimit = Limit;


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
      DEBUG_OUT.printf("%c","0");
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
  while (radio1.available(&pipe)) {
    //DEBUG_OUT.printf("RFRxPacket someting on pipe %i", pipe);
    if (!packetBuffer.full()) {
      //DEBUG_OUT.printf(" reading %i", i++);
      NRF24_packet_t *p = packetBuffer.getFront();
      if (!p) DEBUG_OUT.printf("RFRxPacket packetbuffer getFront full\r\n");
      p->timestamp = micros(); // Micros does not increase in interrupt, but it can be used.
      p->packetsLost = lostPacketCount;
      radio1.setChannel(RxCH);//setChannel(DEFAULT_RECV_CHANNEL);
      uint8_t packetLen = radio1.getPayloadSize();
      if (packetLen > MAX_RF_PAYLOAD_SIZE)
        packetLen = MAX_RF_PAYLOAD_SIZE;
      radio1.read(p->packet, packetLen);
      //DEBUG_OUT.println(F(" payload ok"));
      packetBuffer.pushFront(p);
      if (!p) DEBUG_OUT.printf("RFRxPacket packetbuffer pushFront full\r\n");
      else sts = true;
      lostPacketCount = 0;

      }
    else {
      // Buffer full. Increase lost packet counter.
      if (lostPacketCount < 255){
         lostPacketCount++;
         DEBUG_OUT.printf("RFRxPacket lost packet, full buffer\r\n");
         }
      DEBUG_OUT.printf("RFRxPacket full buffer\r\n");
      sts = false;
      }
    radio1.flush_rx();// Flush buffer to drop the packet.
    }//while
  return sts;
 // DEBUG_OUT.println(F("RFRxPacket END"));
}//----RFRxPacket-----------------------------------------------------------------------------------

#ifdef ESP8266
void ICACHE_RAM_ATTR RFirqHandler() {
#else
void RFirqHandler() {
#endif
//---------------------------------------------------------------------------------------
    radio1.maskIRQ(true, true, true);
    DISABLE_EINT;

    bool tx_ok, tx_fail, rx_ready;                // declare variables for IRQ masks
    radio1.whatHappened(tx_ok, tx_fail, rx_ready); // get values for IRQ masks
    // whatHappened() clears the IRQ masks also. This is required for
    // continued TX operations when a transmission fails.
    // clearing the IRQ masks resets the IRQ pin to its inactive state (HIGH)

    if (rx_ready){
        //DEBUG_OUT.printf ("nrf IRQ Rx ok %i\r\n", rx_ready);
        if (RFRxPacket())
            RFAnalyse();
        isIrq = true;
        }
     else DEBUG_OUT.printf ("nrf IRQ Rx not ok %i\r\n", rx_ready);

    if (tx_ok) DEBUG_OUT.printf ("nrf Tx ok %i ", tx_ok);
    //else DEBUG_OUT.printf ("nrf Tx ok ?? %i ", tx_ok);
    if (tx_fail) DEBUG_OUT.printf ("nrf Tx fail %i ", tx_fail);
    //else DEBUG_OUT.printf ("nrf Tx fail ?? %i ", tx_fail);


//  if (tx_fail)            // if TX payload failed
//        radio1.flush_tx(); // clear all payloads from the TX FIFO

  radio1.maskIRQ(true, true, false);
  ENABLE_EINT;

}//---RFirqHandler-------------------------------------------------------------------

void   setRxPipe(void){
//---------------------------------------------------------------------------------------

  if (SNIFFER){
     radio1.openReadingPipe(0, 0x00aa);
     radio1.openReadingPipe(1, 0x0055);
     }
  else {
      //radio1.openReadingPipe(0, DTU_RADIO_ID);
      radio1.openReadingPipe(1, DTU_RADIO_ID);
      //radio1.openReadingPipe(1, WR1_RADIO_ID);
      }

}//----setRxPipe----------------------------------------------------------------------

static void RFConfig(void) {
//---------------------------------------------------------------------------------------

  while (!radio1.begin()) {
    DEBUG_OUT.printf("Radio hardware is not responding!!\r\n");
    delay(1000);
    }

  radio1.setAutoAck(0);
  radio1.setRetries(0, 0);

  radio1.setDataRate(DEFAULT_RF_DATARATE);
  radio1.disableCRC();
  // Use lo PA level, as a higher level will disturb CH340 DEBUG_OUT usb adapter
  radio1.setPALevel(PA_LEVEL);
  radio1.setPayloadSize(MAX_RF_PAYLOAD_SIZE);
  radio1.setAddressWidth(5);

  setRxPipe();
  // We want only RX irqs,
  if(INTERRUPT){
    // maskIRQ args = "data_sent", "data_fail", "data_ready"
    //Configuring IRQ pin to reflect data_ready events
    radio1.maskIRQ(true, true, false);
    //Configuring IRQ pin to reflect all events
    //radio1.maskIRQ(false, false, false);
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

  radio1.printDetails();
  delay(1000);
  UpdateTxMsgTick = millis() + 1000;
}//--RFConfig-------------------------------------------------------------------------------------

void setup(void) {
//---------------------------------------------------------------------------------------

  DEBUG_OUT.begin(SER_BAUDRATE);

  DEBUG_OUT.flush();
  DEBUG_OUT.printf(".....................\r\n");
  DEBUG_OUT.printf("DTU-Simulation for MI \r\n");
  DEBUG_OUT.printf(".....................\r\n");
  DEBUG_OUT.printf("Begin Setup  wait....\r\n");
  if (WR_LIMITTED) ZEROEXP = 1;
  delay(2000);
  #ifdef ESP8266
    if (WITHWIFI){
      if(!SNIFFER){
          while (!setupWifi())
            DEBUG_OUT.printf("Setup Wifi.......\r\n");

          DEBUG_OUT.printf("Setup clock .....\r\n");
          setupClock();
          DEBUG_OUT.printf("Setup mqtt ......\r\n");
          setupMQTT();
          DEBUG_OUT.printf("Setup www .......\r\n");
 //         setupWebServer();
                 // setupUpdateByOTA();
          calcSunUpDown (getNow());
          istTag = isDayTime(0);
          DEBUG_OUT.printf ("Es ist %s \r\n",(istTag?"Tag":"Nacht"));
                //hmPackets.SetUnixTimeStamp (getNow());
         }
      }
  #else
    //hmPackets.SetUnixTimeStamp(0x62456430);
  #endif
  delay(2000);
  //---NRF--------------------------
  DEBUG_OUT.printf("Setup NRF......\r\n");
  // Configure nRF IRQ input
  if(INTERRUPT)
    pinMode(RF1_IRQ_PIN, INPUT);
  RFConfig();

  delay(1000);
  UpdateMqttTick = millis() + 200;
  if (WITHWIFI)
      STARTTIME=(String)getDateStr(getNow())+" "+(String)getTimeStr(getNow());

  if (MI300) strcpy(MIWHAT,"MI-300");
  if (MI600) strcpy(MIWHAT,"MI-600");
  if (MI1500) strcpy(MIWHAT,"MI-1500");
  DEBUG_OUT.printf("Microinverter is %s, starting at ",MIWHAT);
  DEBUG_OUT.println(STARTTIME); //cant print (String) with printf???
  TxLimitSts=0;

  DEBUG_OUT.printf("Version %s \r\n",VERSION);
  DEBUG_OUT.printf("Setup finished --------Type 1  for HELP-----------\r\n");


}//---setup------------------------------------------------------------------------------------


static void RFTxPacket(uint64_t dest, uint8_t *buf, uint8_t len) {
//----------------------------------------------------------------------------------------------------

  radio1.flush_tx();

  if (DEBUG_TX_DATA) {
      DEBUG_OUT.printf("Send... CH%02i",TxCH);
       //packet buffer to output
      for (uint8_t i = 0; i < len; i++){
        if (buf[i]==0){DEBUG_OUT.printf("%s","00");}
        else { if (buf[i]<0x10) {DEBUG_OUT.printf("%c","0");}
             DEBUG_OUT.printf("%x",buf[i]);
             }
        }
      if (!DEBUG_TX_DATA) DEBUG_OUT.println();
      }
  //if(INTERRUPT) DISABLE_EINT;//?????
  radio1.stopListening(); //***************************************
  radio1.setCRCLength(RF24_CRC_16);
  radio1.enableDynamicPayloads();
  radio1.setAutoAck(true);
  radio1.setRetries(3, 15); //5,15
  radio1.openWritingPipe(dest);
  radio1.setChannel(TxCH);
  //if(INTERRUPT) ENABLE_EINT;//?????
  uint8_t res = radio1.write(buf, len);
  if (DEBUG_TX_DATA)
     DEBUG_OUT.printf("..... res: %i\r\n", res);

   //radio1.print_status(radio1.get_status());

  // Try to avoid zero payload acks (has no effect)
  radio1.openWritingPipe(DUMMY_RADIO_ID);
  radio1.setAutoAck(false);
  radio1.setRetries(0, 0);
  radio1.disableDynamicPayloads();
  radio1.setCRCLength(RF24_CRC_DISABLED);

  radio1.setChannel(RxCH);
  radio1.startListening();

}//----RFTxPacket-------------------------------------------------------------------------------------------------------

void HopCH(void){
//----------------------------------------------------------------------------------------------------

      if ( (millis() - timeLastAck) > RxAckTimeOut){  //hop RxCH when timeout RXack
         RxChId++;
         if (RxChId >= sizeof(channels))// / sizeof(channels[0]) )
           RxChId = 0;
         RxCH = channels[RxChId];
         timeLastAck = millis();
         RxAckTimeOut = 100; //try to find out a channel faster
         }
     //tx allways hopping
     TxChId++;
     if (TxChId >= sizeof(channels))// / sizeof(channels[0]) )
       TxChId = 0;
     TxCH = channels[TxChId];

}//----HopCH----------------------------------------------------------------------------------------


void SerialCmdHandle(void){
//----------------------------------------------------------------------------------------------------------------------
    switch (SerCmd){
        case 1:
          DEBUG_OUT.printf("\r\n\r\n\r\n1: help\t\t2: Status\r\n");
          DEBUG_OUT.printf("3:PA_LOW\t4:PA_HIGH\t5:PA_MAX\r\n");
          DEBUG_OUT.printf("6:Sniffer\t7:ZeroEx\r\n");
          DEBUG_OUT.printf("8:OnlyRX\t9:ShowTX\t13:ShowRX\r\n");
          DEBUG_OUT.printf("10:Wifi\t\t11:CRC\t\t12:reboot\t14:IRQ\r\n");
          DEBUG_OUT.printf("20:WRinfo\t30-39:RxAckTmo\t100-1999:limiting(W)\r\n\r\n\r\n");
          SerCmd=0; //stop command
        break;
        case 2:
          DEBUG_OUT.printf("\r\n\r\nVersion\t %s started at ",VERSION);
          DEBUG_OUT.println(STARTTIME); //cant print string with printf???
          DEBUG_OUT.printf("DEBUG_RCV_DATA \t%i\r\n",DEBUG_RCV_DATA);
          DEBUG_OUT.printf("DEBUG_TX_DATA \t%i\r\n",DEBUG_TX_DATA);
          switch (PA_LEVEL){
              case 1:DEBUG_OUT.printf("PA_LEVEL_LOW \t%i\r\n",PA_LEVEL);
              break;
              case 2:DEBUG_OUT.printf("PA_LEVEL_HIGH \t%i\r\n",PA_LEVEL);
              break;
              case 3:DEBUG_OUT.printf("PA_LEVEL_MAX \t%i\r\n",PA_LEVEL);
              break;
              }
          DEBUG_OUT.printf("WITHWIFI \t%i\r\n",WITHWIFI);
          DEBUG_OUT.printf("ZEROEXP \t%i\r\n",ZEROEXP);
          DEBUG_OUT.printf("INTERRUPT \t%i\r\n",INTERRUPT);
          DEBUG_OUT.printf("SNIFFER \t%i\r\n",SNIFFER);
          DEBUG_OUT.printf("ONLY_RX \t%i\r\n",ONLY_RX);
          DEBUG_OUT.printf("INTERRUPT \t%i\r\n",INTERRUPT);
          DEBUG_OUT.printf("CHECK_CRC \t%i\r\n",CHECK_CRC);
          DEBUG_OUT.printf("WITHMQTT \t%i\r\n",WITHMQTT);
          DEBUG_OUT.printf("TIMEOUTRXACK \t%i msek\r\n\r\n",RxAckTimeOut);
          DEBUG_OUT.print("MY IP "); DEBUG_OUT.println(PrintMyIP());DEBUG_OUT.println();//cant print (String) with printf???
          SerCmd=0; //stop command
        break;
        case 3:
           PA_LEVEL = RF24_PA_LOW;
           radio1.setPALevel(PA_LEVEL);
           DEBUG_OUT.printf("RF24_PA_LOW\r\n");
           radio1.printDetails();
           SerCmd=0; //stop command
        break;
        case 4:
          PA_LEVEL = RF24_PA_HIGH;
          radio1.setPALevel(PA_LEVEL);
          DEBUG_OUT.printf("RF24_PA_HIGH\r\n");
          radio1.printDetails();
          SerCmd=0; //stop command
        break;
        case 5:
           PA_LEVEL = RF24_PA_MAX;
           radio1.setPALevel(PA_LEVEL);
           DEBUG_OUT.printf("RF24_PA_MAX\r\n");
           radio1.printDetails();
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
              setup();
              SerCmd=0; //stop command
        break;
        case 11:
              CHECK_CRC = (CHECK_CRC) ? 0 : 1;
              DEBUG_OUT.printf("CMD CHECK_CRC %i\r\n",CHECK_CRC);
              SerCmd=0; //stop command
        break;
        case 12:
              setup();
              SerCmd=0; //stop command
        break;
        case 13:
              DEBUG_RCV_DATA = (DEBUG_RCV_DATA) ? 0 : 1;
              DEBUG_OUT.printf("CMD DEBUG_RCV_DATA %i\r\n",DEBUG_RCV_DATA);
              SerCmd=0; //stop command
        break;
        case 14:
              INTERRUPT = (INTERRUPT) ? 0 : 1;
              setup();
              DEBUG_OUT.printf("CMD INTERRUPT %i\r\n",INTERRUPT);
              SerCmd=0; //stop command
        break;

        case 31 ... 39: //set new RxAckTimeOut 1000,2000,3000,4000,5000 ....
              RxAckTimeOut = (SerCmd-30)*1000;
              DEBUG_OUT.printf("CMD TIMOACK %i\r\n",RxAckTimeOut);
              SerCmd=0; //stop command
        break;

        default:
        //there are other commands for Tx
        break;
        }

}
//----SerialCmdHandle-----------------------------------------------------------------------------------------------------

void RFisTime2Send (void) {
//----------------------------------------------------------------------------------------------------------------------
  static uint8_t MIDataCMD = 0x36;   //begin with first PV
  static uint8_t MI600_DataCMD = 0x09 ;
  static uint8_t telegram = 0;
  int32_t size = 0;
  uint64_t dest = WR1_RADIO_ID;
  uint8_t UsrData[10];
  char Cmd = 0;

  if (millis() >= UpdateTxMsgTick) {
    UpdateTxMsgTick += TXTIMER;

    if (telegram > sizeof(channels))    telegram = 0;

    if (MI300) {        // 1 PV
        MIDataCMD=0x09;
        }
    if (MI600){         // 2 PVs
        if (MI600_DataCMD == 0x09) MI600_DataCMD=0x11; //flipflop
        else if (MI600_DataCMD == 0x11) MI600_DataCMD=0x09;
        MIDataCMD=MI600_DataCMD;
        }
    if (MI1500){        // 4 PVs
        if (MIDataCMD > 0x0039) MIDataCMD= 0x0036;
        }

    switch(telegram) {
      case 0:
        //set SubCmd and  UsrData Limiting
        if ((TxLimitSts) && (abs (GridPower) > TOLERANCE)) {        //Limitierung ????chek it again
          Cmd=0x51;
          DEBUG_OUT.printf("CMD:%03X CH:%i Sts:%4i Setting Limit:%i\r\n",Cmd, TxCH,TxLimitSts,Limit);
          UsrData[0]=0x5A;UsrData[1]=0x5A;UsrData[2]=100;//0x0a;// 10% limit
          UsrData[3]=((Limit*10) >> 8) & 0xFF;   UsrData[4]= (Limit*10)  & 0xFF;   //WR needs 1 dec= zB 100.1 W
          size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,5);
          //Limit=0; will be set after ack limiting
          }
        else {
          TxLimitSts = 0;
          UsrData[0]=0x0;//set SubCmd and  UsrData for data request
          size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, MIDataCMD, UsrData,1);
          }
        break;
      case 1:
        switch (SerCmd){ //SrCmd Parts to send
           case 20: //request WR info
              Cmd=0x0f;
              UsrData[0]=0x01;
              DEBUG_OUT.printf("CH:%i CMD 0x%X Sending WRInfo Req:0x%X\r\n",TxCH,Cmd,WRInfo);
              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,1);
            break;
//            case 21: //GONGFA
//              Cmd=0x02;
//              UsrData[0]=0x0;
//              DEBUG_OUT.print(F("CMD 0x"));DEBUG_OUT.print(Cmd,HEX);DEBUG_OUT.println(F(" Sending Gongfa"));
//              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,1);
//            break;
            default://request WR data
              UsrData[0]=0x0;//set SubCmd and  UsrData
              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, MIDataCMD, UsrData,1);
            }
        break;
      default: //request WR data
        UsrData[0]=0x0;//set SubCmd and  UsrData
        size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, MIDataCMD, UsrData,1);
      }  //switch telegram

    RFTxPacket(dest, (uint8_t *)&sendBuf, size);
    telegram++;
    MIDataCMD++;
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
        printf_P(PSTR("%04X "), crc);
        if (((crc >> 8) != p->packet[payloadLen + 2]) || ((crc & 0xFF) != p->packet[payloadLen + 3]))
          DEBUG_OUT.printf("%i",0);
        else
          DEBUG_OUT.printf("%i",1);
        }
      else {
        DEBUG_OUT.printf("remain %i ",remain);
        }
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
     DEBUG_OUT.printf("PVcheck timer %i\r\n",timeCheckPV);
     return 0;
     }

  switch (NRofPV){
  case 1:
     if (pvCnt[0]==NRofPV) return NRofPV;
  break;
  case 2:
     if ((pvCnt[0]+pvCnt[1])==NRofPV) return NRofPV;
  break;
  case 3:
     if ((pvCnt[0]+pvCnt[1]+pvCnt[2])==NRofPV) return NRofPV;
  break;
  case 4:
     if ((pvCnt[0]+pvCnt[1]+pvCnt[2]+pvCnt[3])==NRofPV) return NRofPV;

  break;
  }
  return 0;
}//----------------------------------------------------------------------------------------------

void HandleSerialCmd(void){ //read from serial for WR control cmd's
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

      if ((temporary >1999) || (temporary < 100)){  //other cmds. implemented > 2000, request WR info etc.
        SerCmd = temporary;  //this is a serial command
        TxLimitSts=0;      //no nrf send needed
        //DEBUG_OUT.printf("TxLimitSts serialcmd-0 ---- %i ",TxLimitSts);
        SerialCmdHandle();
        }
      else{
        Limit = temporary; //Limit is 100-1999
        TxLimitSts=1; //nrf send needed
        //DEBUG_OUT.printf("TxLimitSts serialcmd-1 ---- %i ",TxLimitSts);
        }
      DEBUG_OUT.printf(" SerialIn: %s Cmd:%i\r\n",SerialIn,temporary);
      InCnt=0;

      }
  }
}//---HandleSerialCmd-------------------------------------------------------------------------------

void SendMQTTMsg(String topic, String value){
//-------------------------------------------------------------------------------------------
  if (WITHWIFI && MQTT){
      if (!checkWifi()) return;
      mqttClient.beginMessage(topic);
      mqttClient.print(value);
      mqttClient.endMessage();
      }

}//---SendMQTTMsg-------------------------------------------------------------------------------------------------------

void PrintOutValues(void){
//----------------------------------------------------------------------------------------------------------------------
  DEBUG_OUT.printf("CH:%02i MI:%04iW [PV%1i %5sW %4sV %4sA %04iWh][%5sV %4sHz %4sC S:%i] Grd:%04iW Lm:%04iW PVok:%i  ",
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
  Limit,
  PVcheck());

  uint64_t t=millis();
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

  if ((30<U_DC<50) && (0<I_DC<15) && (200<U_AC<300) && (45<F_AC<55) && (0<P_DC<420) && (0<TEMP<80))
   DataOK = 1;
  else { DEBUG_OUT.printf("Wrong data!!\r\n");DataOK =0; return;}

  STAT = (uint8_t)(p->packet[25] );
  FCNT = (uint8_t)(p->packet[26]);
  FCODE = (uint8_t)(p->packet[27]);

  if (p->packet[2] == 0xB6)  {PV= 0; TotalP[1]=P_DC; pvCnt[0]=1;}//port 1
  if (p->packet[2] == 0xB7)  {PV= 1; TotalP[2]=P_DC; pvCnt[1]=1;}//port 2
  if (p->packet[2] == 0xB8)  {PV= 2; TotalP[3]=P_DC; pvCnt[2]=1;}//port 3
  if (p->packet[2] == 0xB9)  {PV= 3; TotalP[4]=P_DC; pvCnt[3]=1;}//port 4
  TotalP[0]=TotalP[1]+TotalP[2]+TotalP[3]+TotalP[4];//in TotalP[0] is the totalPvW
  if((P_DC>PVPOWER) || (P_DC<0) || (TotalP[0]>MAXPOWER)){// cant be!!
    DEBUG_OUT.printf("Wrong Data.. PV%1i %5sW Total:%5sW \r\n", PV,String(P_DC,1),String(TotalP[0],1) );
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
}//--MI600StsMsg---------------------------------------------------------------------

void MI600DataMsg(NRF24_packet_t *p){
  //--------------------------------------------------------------------------------------------------------------------
  U_DC =  (float) ((p->packet[11] << 8) + p->packet[12])/10;
  I_DC =  (float) ((p->packet[13] << 8) + p->packet[14])/10;
  U_AC =  (float) ((p->packet[15] << 8) + p->packet[16])/10;
  F_AC =  (float) ((p->packet[17] << 8) + p->packet[18])/100;
  P_DC =  (float)((p->packet[19] << 8) + p->packet[20])/10;
  Q_DC =  (float)((p->packet[21] << 8) + p->packet[22])/1;
  TEMP =  (float) ((p->packet[23] << 8) + p->packet[24])/10;

  if ((30<U_DC<50) && (0<I_DC<15) && (200<U_AC<300) && (45<F_AC<55) && (0<P_DC<420) && (0<TEMP<80))
   DataOK = 1;  //we need to check this, if no crc
  else { DEBUG_OUT.printf("Data Wrong!!\r\n");DataOK =0; return;}

  if (p->packet[2] == 0x89)  {PV= 0; TotalP[1]=P_DC; pvCnt[0]=1;}//port 1
  if (p->packet[2] == 0x91)  {PV= 1; TotalP[2]=P_DC; pvCnt[1]=1;}//port 2

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

  switch (p->packet[2])  {

    case 0xD1:
      TxLimitSts=0;//stop sending Limit
      DEBUG_OUT.printf("Limiting(0x51) is ok CMD:%X  RxCH:%i   TxLimitSts ack ---- %i\r\n",p->packet[2],RxCH,TxLimitSts);
      timeLastAck = millis();
      RxAckTimeOut = TIMEOUTRXACK;
      pvCnt[0]=pvCnt[1]=pvCnt[2]=pvCnt[3]=0; //reset PV;sts
      break;
//    case 0x82: //Gongfa
//      DEBUG_OUT.print (F("Gongfa(0x2) is ok CMD="));
//      DEBUG_OUT.println(p->packet[2], HEX);
//      dumpData(&p->packet[3], payloadLen);
//      SerCmd=0; //stop sending
//      break;
    case 0x8f:
      DEBUG_OUT.printf("WRInfo ack %x is ok for CMD 0x0F \r\n",p->packet[2]);
      DEBUG_OUT.printf("WR %x:%x:%x:%x ",p->packet[7],p->packet[8],p->packet[9],p->packet[10]);
      DEBUG_OUT.printf("HWPN: %i.%i ",p->packet[11],p->packet[12]);
      DEBUG_OUT.printf("HWVers: %i.%i ",p->packet[13],p->packet[14]);
      DEBUG_OUT.printf("APPFVers: %i.%i ",p->packet[15],p->packet[16]);
      DEBUG_OUT.printf("GPFCode: %i.%i ",p->packet[17],p->packet[18]);
      DEBUG_OUT.printf("GPFVers: %i.%i\r\n",p->packet[19],p->packet[20]);
      SerCmd=0; //stop sending WRInfo
      timeLastAck = millis();
      break;
    case 0xB6:    //4 ports
    case 0xB7:    //4 ports
    case 0xB8:    //4 ports
    case 0xB9:    //4 ports
        timeLastAck = millis();
        RxAckTimeOut = TIMEOUTRXACK;
        MI1500DataMsg(p);
        break;

    case 0x89:    //1-2 ports
    case 0x91:    //2 ports
        timeLastAck = millis();
        RxAckTimeOut = TIMEOUTRXACK;
        MI600DataMsg(p);
        break;

    case 0x88:    //1-2 ports
    case 0x92:    //2 ports
        timeLastAck = millis();
        RxAckTimeOut = TIMEOUTRXACK;
        MI600StsMsg(p);
        break;
    default:
       DEBUG_OUT.printf("New CMD  %x \t",p->packet[2]);
       RFDumpRxPacket (p, payloadLen); //output received data
    }
}//--MIAnalysePacket----------------------------------------------------------------------------------

void RFAnalyse(void) {
//--------------------------------------------------------------------------------------------------
 int i=0;
//  if (packetBuffer.empty()){
//  DEBUG_OUT.println("RFAnalyse buffer empty");
//  return;
//  }
  while (!packetBuffer.empty()) {
    //DEBUG_OUT.printf("Analyse read buffer %i\r\n",i++);
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

    if ( (DEBUG_RCV_DATA) || (SNIFFER) )
      RFDumpRxPacket (p, payloadLen); //output received data


    if (CHECK_CRC) {
      // If CRC is invalid only show lost packets
      if (((crc >> 8) != p->packet[payloadLen + 2]) || ((crc & 0xFF) != p->packet[payloadLen + 3])) {
        if (p->packetsLost > 0) {
          DEBUG_OUT.printf("RFAnalyse CRC lost packets: %i",p->packetsLost);
          }
        //DEBUG_OUT.printf("RFAnalyse CRC fail "));
        packetBuffer.popBack();
//        packetBuffer.clear();//????????
//        radio1.flush_rx();// Flush buffer to drop the packet.
        continue;
        }
      // Dump a decoded packet only once
      if (lastCRC == crc) {
//
//        DEBUG_OUT.printf("RFAnalyse last CRC "));
        packetBuffer.popBack();
//        packetBuffer.clear();//????????
//        radio1.flush_rx();// Flush buffer to drop the packet.
        continue;
        }
      lastCRC = crc;
    }// if checkcrc
    // Don't dump mysterious ack packages
    if (payloadLen == 0) {
      packetBuffer.popBack();
      DEBUG_OUT.printf("RFAnalyse mysterious ack \r\n");
      continue;
      }

    if (p->packetsLost > 0) {
      DEBUG_OUT.printf("RFAnalyse packet lost: %i\r\n", p->packetsLost);
      continue;
      }
    if (!SNIFFER)
      MIAnalysePacket(p,payloadLen);

    // Remove record as we're done with it.
    packetBuffer.popBack();
  } //while
//  packetBuffer.popBack();??????????????
}//-----RFAnalyse-------------------------------------------------------------------------------


void DoZeroExport(void){
//-------------------------------------------------------------------------------------------------

    if (millis() < UpdateZeroExpTick){//not overload the web&mqtt server
        DEBUG_OUT.printf("ZeroExport timer: not yet !\r\n");
        return; //not in first 60 sek
        }
    else if (WR_LIMITTED) UpdateZeroExpTick += FIXLIMITTICK;//set timer a bit longer
         else UpdateZeroExpTick += ZEXPUPDATETICK;
    if (WR_LIMITTED){
        Limit = WR_LIMITTED;
        GridPower = TOLERANCE+1; //;-)
        DEBUG_OUT.printf("Limiting WR %i\r\n", Limit);
        TxLimitSts=1;
        return;
        }
    if (!TxLimitSts){
        if (abs (GridPower) > TOLERANCE) { // || (abs (Limit-OldLimit) > TOLERANCE )){//if change more than 15 watt
            DEBUG_OUT.printf("Grid out of Tolerance %f\r\n", GridPower);
            if (GridPower >0){ //Exporting is PLUS zu viel power
                Limit= PMI - GridPower;  //327-(296)
                }
            else { //importing is minus zu wenig power
                Limit= PMI+ abs(GridPower); //327+abs(-296)
                }
            if (HH > 17) Limit = 160; //after 17h not shut down the mi
            else if (Limit < MINPOWER) Limit = 100;
                 else if (Limit > MAXPOWER) Limit = MAXPOWER;
            if (!TxLimitSts){
                //OldLimit = Limit;
                TxLimitSts=1; //we can send
                //DEBUG_OUT.printf("TxLimitSts zeroexp-0 ---- %i\r\n",TxLimitSts);
                }
            }
        else {
             TxLimitSts=0;

            DEBUG_OUT.printf("Grid in the tolerance %i\r\n", (int)GridPower);
            //DEBUG_OUT.printf(" TxLimitSts zeroexp-1 ---- %i\r\n",TxLimitSts);
            }
    }
}//---DoZeroExport----------------------------------------------------------------------------------

void loop(void) {
//===============================================================================================

  HopCH();
  //if (INTERRUPT) DISABLE_EINT;
  //setRxPipe();
  radio1.setChannel(RxCH);
  radio1.startListening();
  //if (INTERRUPT) ENABLE_EINT;


  if(!INTERRUPT){//polling if !INTERRUPT
    if (RFRxPacket())
        RFAnalyse();
    }
  if (! SNIFFER){
    HandleSerialCmd();  //read from serial any command
    if ((!ONLY_RX) && (istTag)) //  weiter machen!!!!
        RFisTime2Send();//SEND PACKET
    }
  PVcheck();
  #ifdef ESP8266
    if (WITHWIFI && (!SNIFFER)){
       if (!checkWifi()){
         setup();
         //checkUpdateByOTA();
         }
       if (millis() >= UpdateMqttTick){//not overload the web&mqtt server
         istTag = isDayTime(0);
         UpdateMqttTick += IPServicesUPDATETICK;
         if (MQTT) mqttClient.poll();
         if (PVcheck()){
             if (ZEROEXP)//not on first 60sek
                DoZeroExport();

 //            PVcheck(true); //reset checkPVs
             webserverHandle();
             if (MQTT && DataOK ){
               //DEBUG_OUT.printf("Update Mqtt Services\r\n");
               mqttClient.poll();
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
               }
             else {
               //DEBUG_OUT.printf("No MQTT..\r\n");
               if (!MQTT) MQTT=setupMQTT();
               }
             }//server
         }//update
         //UpdtCnt=0;
     }//wifi
  #endif //esp8266

}//-----loop-----------------------------------------------------------------------------------------
