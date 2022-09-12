/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Channel scanner
 *
 * Example to detect interference on the various channels available.
 * This is a good diagnostic tool to check whether you're picking a
 * good channel for your application.
 *
 * Inspired by cpixip.
 * See http://arduino.cc/forum/index.php/topic,54795.0.html
 */

/* modification by Ziyat T.
for the Project initiated here: https://www.mikrocontroller.net/topic/525778
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//#include "printf.h"
//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus CE and CSN
// CE  - Chip Enable
// CSN - Chip Select
#ifdef ESP8266
  #define CE_PIN  (D3)
  #define CSN_PIN  (D8)
  #define RF1_IRQ_PIN (D4)
#else
  #define CE_PIN  9
  #define CSN_PIN 10
#endif
RF24 radio(CE_PIN,CSN_PIN);
bool INTERRUPT = true;

//
// Channel info
//

const uint8_t num_channels = 126;
//uint8_t values[num_channels];
//uint8_t weak[num_channels];
uint64_t add;


// define WR address straight and reversed
//uint64_t address =0x6370716001;//   6071706301; //WR
//uint64_t address2=0x7222841201;//  0x1284227201; //DTU
////uint64_t address2=0x6071706301;
//int width = 5;


// define preamble as address to sniff any traffic
uint64_t address2=0x0055;
uint64_t address=0x00aa;
int width = 2;
//#define ALLCH
#ifdef ALLCH
    int rcvChannels[125];  //2400 to 2525 MHz (MegaHz). The nRF24L01 channel spacing is 1 Mhz which gives 125 possible channels numbered 0 .. 124
#else
    int rcvChannels[]  = {1,3,6,9, 11, 23,35, 40, 61,70, 75};//{1, 3, 6, 9, 11, 23, 40, 61, 75};
#endif

int     NRofCH = std::size(rcvChannels);//sizeof(rcvChannels)/sizeof(rcvChannels[0]);  
int     RxCH = 0 ;  //mit CH 3 anfangen
int     cntCH = 0;

bool isRxIrq = false;

//void* data;

//const int num_reps = 1;
uint8_t pipeNum;
//uint8_t RxData = 0;
//int car=2;
//bool goodSignal = false;
byte    data[32];


#ifdef ESP8266
    #define DISABLE_EINT noInterrupts()
    #define ENABLE_EINT  interrupts()
#else     // für AVR z.B. ProMini oder Nano 
  #define DISABLE_EINT EIMSK = 0x00
  #define ENABLE_EINT EIMSK = 0x01
#endif

void RFanalyse (){//----IRQ-------------------------------------------------------------------------
  while (radio.available(&pipeNum)){ 
    (pipeNum == 0) ? add=address : add=address2;
    memset(data,0x00,sizeof(data));
    radio.read(&data,sizeof(data));

    //check if byte 4,5,6 defines a valid pl-length
    byte paylSize[2+1];
    bool paylSize_ok[2+1];
    int pid[2+1];
    int noack[2+1];
    int g=2;
    for (int g = 0; g < 2+1; g++) {
      paylSize[g] = data[g+3]>>2;
      paylSize_ok[g] = ((paylSize[g] >=0) && (paylSize[g]<= 32));
      pid[g] = data[g+3] & 3;
      noack[g]=data[g+4]>>7;
      }

    //repeat this line, move payload 1 bit to see payload after no_ack flag
    if ((paylSize_ok[2]) && (sizeof(data) <=32)) { //payload size ok
      Serial.printf("CH:%03i P:%i ", rcvChannels[RxCH],pipeNum);
      
      for (int d = 0; d < 4; d++) { Serial.printf("%02x", data[d]); } //dest. adress
      Serial.print(" ");    
      
      Serial.printf("(%02i/%i/%i) ", paylSize[2],pid[2],noack[2]);

      for (int d = 6; d < sizeof(data); d++) {   //data
          byte shifted = data[d]<<1;
          if (data[d+1]>>7) {
              bitSet(shifted,0);
          }                      
          Serial.printf("%02x", shifted); //write data
          switch (d){
              case 6: Serial.print(" ("); break;
              case 10: Serial.print("|"); break;
              case 14: Serial.print(") "); break;
            }
          
          if (d==6+paylSize[2]-1) Serial.print(" ["); // end of payload
          else if (d==6+paylSize[2]+1) { Serial.print("] "); break;} // end of crc16
                else if (d>14) Serial.print(" ");

      } //for data

      getMillis();
      //RF24::print_status(RF24::get_status());
      //Serial.println(radio.get_status(),HEX);

      //printf("-size:%d, carr:%d time:%ld good:%d 5:%i/%i/%i addr:0x%04x shbit9 ", sizeof(data), car, timestamp, goodSignal, paylSize[2],pid[2],noack[2], add);
      //printf("\n\r - size:%d time:%ld addr:0x%04x shiftbit9 \n\r", sizeof(data), timestamp, add);
    } //if PayLoad ok
  }//if radio.available

  isRxIrq = false;

}//----------------------------------------------------------------------------------------------


#ifdef ESP8266
void ICACHE_RAM_ATTR RFirqHandler() {
#else
void RFirqHandler() {
#endif
//---------------------------------------------------------------------------------------

  DISABLE_EINT;
  radio.maskIRQ(true, true, true); //disable irq if DISABLE_EINT doesnt work ;-)

  bool tx_ok, tx_fail, rx_ready;                // declare variables for IRQ masks
  radio.whatHappened(tx_ok, tx_fail, rx_ready); // get values for IRQ masks
  // whatHappened() clears the IRQ masks also. This is required for
  // continued TX operations when a transmission fails.
  // clearing the IRQ masks resets the IRQ pin to its inactive state (HIGH)

  if (rx_ready) {
    //Serial.printf ("nrf Rx irq %i \r\n", rx_ready);
    RFanalyse();
    isRxIrq = true;
  }
  if (tx_ok) Serial.printf ("nrf Tx ok %i ", tx_ok);
  //else Serial.printf ("nrf Tx ok ?? %i ", tx_ok);
  if (tx_fail) Serial.printf ("nrf Tx fail %i ", tx_fail);
  //else Serial.printf ("nrf Tx fail ?? %i ", tx_fail);
  //if (tx_fail)            // if TX payload failed
  //   radio.flush_tx(); // clear all payloads from the TX FIFO

  radio.maskIRQ(true, true, false);//Configuring IRQ pin to reflect data_ready events
  ENABLE_EINT;

}//---RFirqHandler---------------------------------------------------------------------------------------------------

void getMillis(void){//----------------------------------------------------------------------------------------------
  uint64_t t=millis();
  uint16_t s = (uint16_t) (t / 1000) % 60;
  uint16_t m = (uint16_t) ((t / (1000 * 60)) % 60);
  uint16_t h = (uint16_t) ((t / (1000 * 60 * 60)) % 24);
  uint16_t d = (uint16_t) (t / (1000 * 60 * 60 * 24));
  uint16_t ms = (uint16_t) (t % 1000);
  Serial.printf(" %02i:%02i:%02i:%02i:%02i\r\n",d,h,m,s,ms );
}//------------------------------------------------------------------------------------------------------------------

void setup(void) {//-------------------------------------------------------------------------------------------------
  Serial.begin(115200);
  //printf_begin();

  #ifdef ALLCH
   for(int i = 0; i < NRofCH; i++)  testing all chnnels, put chnr
      rcvChannels[i] = i;
  #endif
  
  delay(4000);
  Serial.printf("\r\n  RF24 scanner Ziyat.t\r\n");
  Serial.printf("==========================\r\n");  
  

  if(INTERRUPT){
    // maskIRQ args = "data_sent", "data_fail", "data_ready"
    radio.maskIRQ(true, true, false);//Configuring IRQ pin to reflect data_ready events
    //Configuring IRQ pin to reflect all events
    //radio.maskIRQ(false, false, false);
    // disable IRQ masking for this step
    //radio.maskIRQ(true, true, true);

    //Attach interrupt handler to NRF IRQ output. Overwrites any earlier handler.
    attachInterrupt(digitalPinToInterrupt(RF1_IRQ_PIN), RFirqHandler, FALLING); // NRF24 Irq pin is active low.
  }

  radio.begin();
    
  radio.setChannel(RxCH);
  radio.setAddressWidth(width);
  radio.setPayloadSize(32);
  radio.setDataRate(RF24_250KBPS); //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS
  radio.setAutoAck(false);
  radio.disableCRC();
  //radio.setCRCLength(RF24_CRC_8); //  RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit

  radio.openReadingPipe(0, address);
  radio.openReadingPipe(1, address2);
  radio.printDetails();
  Serial.printf("\r\n=====wait =====================\r\n"); 
  delay (4000);
  // Get into standby mode
  radio.startListening();
  radio.stopListening();
}//---setup----------------------------------------------------------------------------------------------------------------------

void loop(void) {
//--------------------------------------------------------------------------------------------------------------------
//  memset(values,0,sizeof(values));
//  memset(weak,0,sizeof(weak));

  radio.openReadingPipe(0, address);
  radio.openReadingPipe(1, address2);
  radio.setChannel(rcvChannels[RxCH]);
  radio.startListening();
  Serial.printf ("Start listening\r\n");
  while (true) {
    if (!INTERRUPT){ RFanalyse();}  //we can loop as well
    if (RxCH < (NRofCH-1)) RxCH++; //chanel hop 
    else RxCH=0;
    radio.stopListening();
    radio.setChannel(rcvChannels[RxCH]);
    radio.startListening();
    delay(10);
    //delayMicroseconds(4*32); doesnt work
  } 
} //---loop----------------------------------------------------------------------------------------------

// vim:ai:cin:sts=2 sw=2 ft=cpp
