/*
This software is a QUICK&DIRTY SW for debugging/controlling the Hoymiles inverters over RF(NRF24)
Based on the orig. SW from Hubi's earlier stage from his (https://github.com/hm-soft/Hoymiles-DTU-Simulation)
recoded and expanded for the Hoymiles microinverter family MI, by Ziyat T.

Project initiated here: https://www.mikrocontroller.net/topic/525778
Do not expect any quality from this SW!!!

------------------------------------------------------------------------------------------------------------------------
Configuration are in Settings.h and secrets.h !!
------------------------------------------------------------------------------------------------------------------------
*/
#ifndef __SECRETS_H
#define __SECRETS_H

#define MY_DTU_PRO ((uint64_t)0x1234567801ULL)
#define MY_MI_WR    0x106112345670ULL
#define MY_SSID "mySSID"
#define MY_WIFIPW "myPW"
#define MY_BREITE  11.2866
#define MY_LAENGE  3.3416

#endif //__SECRETS_H