/* die orig. SW ist vom Hubi, wurde von mir(Ziyat T.) für den MI-WR abgeaendert.
Getestet auf ESP8266/ArduinoUNO.
https://www.mikrocontroller.net/topic/525778
https://github.com/hm-soft/Hoymiles-DTU-Simulation
----------------------------------------
Alle Einstellungen sind in Settings.h UND secrets.h !!
----------------------------------------
*/
#ifndef __SONNE_H
#define __SONNE_H

#include <TimeLib.h>

#include "Settings.h"
#include "Debug.h"
#include "wifi.h"


static long SunDown, SunUp;
static uint16_t YYYY, MON, DD, HH, MiN, SEK;

void calcSunUpDown (time_t date) {
//--------------------------------------------------------------------------------------------------------------
    //SunUpDown res = new SunUpDown();
    boolean isSummerTime = true;   // TODO TimeZone.getDefault().inDaylightTime(new Date(date));
    
    //- Bogenma�
    double brad = geoBreite / 180.0 * PI;
    // - H�he Sonne -50 Bogenmin.
    double h0 = -50.0 / 60.0 / 180.0 * PI;
    //- Deklination dek, Tag des Jahres d0
    int tage = 30 * month(date) - 30 + day(date); 
    double dek = 0.40954 * sin (0.0172 * (tage - 79.35));
    double zh1 = sin (h0) - sin (brad)  *  sin(dek);
    double zh2 = cos(brad) * cos(dek);
    double zd = 12*acos (zh1/zh2) / PI;
    double zgl = -0.1752 * sin (0.03343 * tage + 0.5474) - 0.134 * sin (0.018234 * tage - 0.1939);
    //-Sonnenuntergang
    double tsu = 12 + zd - zgl;
    double su = (tsu + (15.0 - geoLaenge) / 15.0); 
    int std = (int)su;
    int minute = (int) ((su - std)*60);
    if (isSummerTime) std++;
    SunDown = (100*std + minute) * 100;
    
    //- Sonnenaufgang
    double tsa = 12 - zd - zgl;
    double sa = (tsa + (15.0 - geoLaenge) /15.0); 
    std = (int) sa;
    minute = (int) ((sa - std)*60);
    if (isSummerTime) std++;
    SunUp = (100*std + minute) * 100;
    //DEBUG_OUT.printf("Sonnenaufgang %i Sonnenuntergang %i\r\n", SunUp,SunDown);
}//--------------------------------------------------------------------------------------------------

void getDate(int tz_offset){
//--------------------------------------------------------------------------------------------------
// we need hour as an integer, in zeroexprot later, lets calculate every date ;-)

    time_t localtime = getNow() + (tz_offset * 60 * 60);

    DD = (localtime / 86400);
    HH = (((localtime - DD * 86400) / 3600) % 24);
    MiN = ((((localtime - DD * 86400) - HH * 3600) / 60) % 60);
    SEK = (((((localtime - DD * 86400) - HH * 3600) - MiN * 60)) % 60);

    /* Get number of 4years to accomodate for leap years */
    uint16_t q_years = (DD / 1461);

    /* Recalculate the number of years */
    YYYY = q_years * 4 + (DD - (q_years * 1461)) / 365;

    /* Deterimne no. of days in the current year */
    uint16_t days_last = (DD - (q_years * 1461)) % 365;
    static uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    /* Fix for leap year */
    if (YYYY % 4 == 0){
        days_in_month[1] = 29;
        days_last++;
        }

    /* Retrace current month */
    MON = 0;
    while (days_last > days_in_month[MON]) {
        days_last -= days_in_month[MON++];
        }
    YYYY= 1970 + YYYY;
    MON= MON + 1;
    DD=days_last;
    DEBUG_OUT.printf("%d-%d-%d  %d:%d:%d ", YYYY, MON ,DD , HH, MiN, SEK);
}//---getDate------------------------------------------------------------------------------------------------


bool isDayTime(int tz_offset) {
//----------------------------------------------------------------------------------------------------------
  const int offset=60*15;// 900 = 15 Minuten, vor Sonnenaufgang und nach -untergang
  time_t jetzt = getNow();
  calcSunUpDown(jetzt);
  getDate(tz_offset);

  long jetztMinuteU = (100 * hour(jetzt+offset) + minute(jetzt+offset)) * 100;
  long jetztMinuteO = (100 * hour(jetzt-offset) + minute(jetzt-offset)) * 100;

  //DEBUG_OUT.printf("jetztU %i SunUp %i   jetztO %i SunDown %i ",jetztMinuteU,SunUp,jetztMinuteO,SunDown);
  if ((jetztMinuteU >= SunUp) &&(jetztMinuteO <= SunDown)){
        DEBUG_OUT.printf("Es ist Tag ! %ih %i\r\n",HH,jetztMinuteO);
        return true;
        }
  else {
        DEBUG_OUT.printf("Es ist Nacht ! %i h\r\n",HH);
        return false;
        }
}//---isDayTime-----------------------------------------------------------------------------------------------


int dayofweek(time_t now, int tz_offset) {
//----------------------------------------------------------------------------------------------------------
	// Calculate number of seconds since midnight 1 Jan 1970 local time
	time_t localtime = now + (tz_offset * 60 * 60);
	// Convert to number of days since 1 Jan 1970
	int days_since_epoch = localtime / 86400;
	// 1 Jan 1970 was a Thursday, so add 4 so Sunday is day 0, and mod 7
	int day_of_week = (days_since_epoch + 4) % 7;

	return day_of_week;
}//---dayofweek-----------------------------------------------------------------------------------------------

#endif
