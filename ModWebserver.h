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

// #################  WebServer & OTA #################

#ifndef __MODWEBSERVER_H
  #define __MODWEBSERVER_H
  #define MODWEBSERVER

#include <ESP8266WebServer.h>
#include "Debug.h"
#include "Settings.h"
#include "Globals.h"


ESP8266WebServer server (WEBSERVER_PORT);

String getMyStyle(String myStyle){
        myStyle = "<style>";
        myStyle += "#myT {font-family: Arial, Helvetica, sans-serif;border-collapse: collapse;width:50%;}";
        myStyle += "#myT td,#myT th {border:1px solid #ddd;padding:8px;}";
        myStyle += "#myT th {padding-top:4px;padding-bottom:4px;text-align:left;background-color:#04AA6D;color:white;}";
        myStyle += "h1,h2,h3,h4,h5,h6{font-family:sans-serif;color:maroon;border-bottom:1px solid rgb(200, 200, 200);}";
        myStyle += "</style>";
        return myStyle;
}

void returnOK () {
//---------------------------------------------------------------------------------------------------------
    server.send(200, F("text/plain"), "");
  }


void returnFail(String msg) {
//---------------------------------------------------------------------------------------------------------
    server.send(500, F("text/plain"), msg + "\r\n");
  }

void handleHelp () {
//---------------------------------------------------------------------------------------------------------
    String htmlStyle;
    String out = "<html><head><title>Hoylmoly MI-DTU / Help</title></head><body>";

    out += getMyStyle(htmlStyle);
 //   out += "<h5>Hoylmoly Version " + String(VERSION) + " started on " + String(STARTTIME) +"</h5><br><br><br>";
    out += "<p style='font-family:Arial, Helvetica, sans-serif;'>Hoylmoly Version:"+String(VERSION)+" started on "+STARTTIME+"</p>";

    out += "<table id='myT'>";
    out += "<tr><th>url</th><th>what</th></tr>";
    out += "<tr><td>/</td><td>MI Data</td></tr>";
    out += "<tr><td>/data</td><td>Data as text</td></tr>";
    out += "<tr><td>:[port+1]/update</td><td>OTA update firmware</td></tr>";
    out += "<tr><td>/reboot</td><td>reboot DTU</td></tr>";

    out += "<tr><th>Configuration parameter</th><th>on/off</th></tr>";
    out += "<tr><td>DEBUG_RCV_DATA</td><td>"+ String(DEBUG_RX_DATA) + "</td></tr>";
    out += "<tr><td>DEBUG_TX_DATA</td><td>"+ String(DEBUG_TX_DATA) + "</td></tr>";
    out += "<tr><td>WITHWIFI</td><td>"+ String(WITHWIFI) + "</td></tr>";
    out += "<tr><td>ZEROEXP</td><td>"+ String(ZEROEXP) + "</td></tr>";
    out += "<tr><td>INTERRUPT</td><td>"+ String(INTERRUPT) + "</td></tr>";
    out += "<tr><td>SNIFFER</td><td>"+ String(SNIFFER) + "</td></tr>";
    out += "<tr><td>ONLY_RX</td><td>"+ String(ONLY_RX) + "</td></tr>";
    out += "<tr><td>CHECK_CRC</td><td>"+ String(CHECK_CRC) + "</td></tr>";
    out += "<tr><td>WITHMQTT</td><td>"+ String(WITHMQTT) + "</td></tr>";
    #ifdef WITH_OTA
    out += "<tr><td>WITH_OTA</td><td>"+ String(1) + "</td></tr>";
    #endif

    switch (PA_LEVEL){
      case 1:out += "<tr><td>PA_LEVEL_LOW</td><td>"+ String(PA_LEVEL) + "</td></tr>";
      break;
      case 2:out += "<tr><td>PA_LEVEL_HIGH</td><td>"+ String(PA_LEVEL) + "</td></tr>";
      break;
      case 3:out += "<tr><td>PA_LEVEL_MAX</td><td>"+ String(PA_LEVEL) + "</td></tr>";
      break;
      }

    out += "</table>";
    out += "</body></html>";
    server.send (200, "text/html", out);

}//---------------------------------------------------------------------------------------------------------


void handleReboot () {
//---------------------------------------------------------------------------------------------------------
    returnOK ();
    ESP.reset();
}//---------------------------------------------------------------------------------------------------------

void handleRoot() {
//---------------------------------------------------------------------------------------------------------
    String htmlStyle;
    String out = "<!DOCTYPE html><html><head><title>Hoylmoly MI-DTU / main</title><meta http-equiv='refresh' content='"+String(REFRESH)+"':URL='";
    out += server.uri()+"'>";
    out += getMyStyle(htmlStyle);
    out += "</head><body>";

    out += "<h2>Hoylmoly Micro-Inverter "+ String(MIWHAT) + "</h2>";
    out += "<h5>"+(String)getDateStr(getNow()) + " "+(String)getTimeStr(getNow()) + "</h5>";
    out += "<table id='myT'>";
    out += "<tr><th>MI P</th><th>-Imp/Exp+</th><th>Limit</th><th>U AC</th><th>Freq</th><th>Temp</th></tr>";
    out += "<tr><td>"+String(PMI) +" W</td><td>"+String(GridPower) + " W</td><td>"+String(LIM) + "</td><td>"+String(U_AC);
    out += " V</td><td>"+String(F_AC) + " Hz</td><td>"+String(TEMP) + " C</td></tr>";
    out += "</table>";

    out += "<table id='myT'>";
    out += "<tr>";
    for (byte i = 0; i < (ANZAHL_VALUES); i++)
        out += "<th>" +  String(getChannelName(i)) + "</th>";  //title
    out += "</tr>";

    for (byte pv=0; pv < NRofPV; pv++){
      out += "<tr>";
      for (byte i = 0; i < (ANZAHL_VALUES); i++)
        if (i==0) out += "<td>" +  String(int(VALUES[pv][i])) + "</td>"; //PVnr must not be displayed as a floatnr
        else out += "<td>" +  String(VALUES[pv][i]) + "</td>";
      out += "</tr>";
      }
    out += "</table>";

    out += "<p style='font-family:Arial, Helvetica, sans-serif; font-size:10'> started on "+STARTTIME+" /"+String(XtimeB)+"<br>";
    out += "now&nbsp"+String((is_Day)?"its day time":"its night time")+"</p>";
    out += "<p style='font-family:Arial, Helvetica, sans-serif; font-size:10'> url/help<br>url/reboot<br>url:[port+1]/update OTA</p>";

    out += "</body></html>";
    server.send (200, "text/html", out);

}//---------------------------------------------------------------------------------------------------------


void handleData () {
//---------------------------------------------------------------------------------------------------------
    String out = "";
    for (byte pv=0; pv < NRofPV; pv++)
      for (int i = 0; i < ANZAHL_VALUES; i++) {
        out += String(getChannelName(i)) + '=' + String (VALUES[pv][i]) + '\n';
      }
    server.send(200, "text/plain", out);

}//---------------------------------------------------------------------------------------------------------


void handleNotFound() {
//---------------------------------------------------------------------------------------------------------
    String message  = "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
      message += " NAME:" + server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
}//---------------------------------------------------------------------------------------------------------


void setupWebServer (void) {
//---------------------------------------------------------------------------------------------------------
    server.on("/",        handleRoot);
    server.on("/reboot",  handleReboot);
    server.on("/data",    handleData);
    server.on("/help",    handleHelp);
    //server.onNotFound(handleNotFound);    wegen Spiffs-Dateimanager

    server.begin();

    DEBUG_OUT.println ("[HTTP] installed");
}//---------------------------------------------------------------------------------------------------------

void webserverHandle() {
//---------------------------------------------------------------------------------------------------------
    server.handleClient();
}//---------------------------------------------------------------------------------------------------------


  // #################  OTA #################

  #ifdef WITH_OTA
    #include <ESP8266HTTPUpdateServer.h>

    ESP8266WebServer httpUpdateServer (UPDATESERVER_PORT);
    ESP8266HTTPUpdateServer httpUpdater;


    void setupUpdateByOTA () {
    //---------------------------------------------------------------------------------------------------------
      httpUpdater.setup (&httpUpdateServer, UPDATESERVER_DIR, UPDATESERVER_USER, UPDATESERVER_PW);
      httpUpdateServer.begin();
      DEBUG_OUT.println (F("[OTA] installed"));
    }

    void checkUpdateByOTA() {
    //---------------------------------------------------------------------------------------------------------
      httpUpdateServer.handleClient();
    }
  #endif  //WITH_OTA


#endif //__MODWEBSERVER_H
