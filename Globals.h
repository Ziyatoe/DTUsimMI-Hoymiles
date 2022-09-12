/*
This software is a QUICK&DIRTY SW for debugging/controlling the Hoymiles inverters over RF(NRF24)
Based on the orig. SW from Hubi's earlier stage from his (https://github.com/hm-soft/Hoymiles-DTU-Simulation)
recoded and expanded for the Hoymiles microinverter family MI, by Ziyat T.

Project initiated here: https://www.mikrocontroller.net/topic/525778
Do not expect any quality from this SW!!!

------------------------------------------------------------------------------------------------------------------------
Configuration are  in Settings.h and secrets.h !!
------------------------------------------------------------------------------------------------------------------------
*/
#ifndef __GLOBALS_H
#define __GLOBALS_H

static String STARTTIME="xx";
static char MIWHAT[10];
static bool is_Day = true;
static float  U_DC =0;
static float  I_DC =0;
static float  U_AC =0;
static float  F_AC =0;
static float  P_DC =0;
static float  Q_DC =0;
static float  TEMP =0;

static int      STAT  = 0;
static int      FCNT  = 0;
static int      FCODE = 0;
static uint8_t  PV    = 0;
static uint16_t PMI   = 0;
static uint16_t LIM   = 0; //for ModWebserver
static bool  isMQTT  = false; //Check mqtt ok?
static float   GridPower     = 0;

#ifdef ESP8266
    char * getChannelName (uint8_t i);
    static const int    ANZAHL_VALUES         = 8;
    static float        VALUES[4][ANZAHL_VALUES] = {};
    static const char   *CHANNEL_NAMES[ANZAHL_VALUES]
       = {"PanelNr ",
          "P [W]   ",
          "Udc [V] ",
          "Idc [A] ",
          "E [Wh]  ",
          "Status  ",
          "FCnt    ",
          "FCode   "};
#endif

static int XtimeB = 0; //how many times do the esp boot

#endif __GLOBALS_H

