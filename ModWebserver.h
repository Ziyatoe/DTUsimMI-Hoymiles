/* die orig. SW ist vom Hubi, wurde von mir(Ziyat T.) für den MI-WR abgeaendert.
Getestet auf ESP8266/ArduinoUNO.
https://www.mikrocontroller.net/topic/525778
https://github.com/hm-soft/Hoymiles-DTU-Simulation
----------------------------------------
Alle Einstellungen sind in Settings.h UND secrets.h !!
----------------------------------------
*/

// #################  WebServer #################

#ifndef __MODWEBSERVER_H
  #define __MODWEBSERVER_H
  #define MODWEBSERVER

#include <ESP8266WebServer.h>
#include "Debug.h"
#include "Settings.h"

ESP8266WebServer server (WEBSERVER_PORT);


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
    String out = "<html>";
    out += "<body><h2>Hilfe</h2>";
    out += "<br><br><table>";
    out += "<tr><td>/</td><td>zeigt alle Messwerte in einer Tabelle; refresh alle 10 Sekunden</td></tr>";
    out += "<tr><td>/data</td><td>zum Abruf der Messwerte in der Form Name=wert</td></tr>";
    out += "<tr><td>:{port+1}/update</td><td>OTA</td></tr>";
    out += "<tr><td>/reboot</td><td>startet neu</td></tr>";
    out += "</table></body></html>";
    server.send (200, "text/html", out);
  }


void handleReboot () {
//---------------------------------------------------------------------------------------------------------
    returnOK ();
    ESP.reset();
  }

void handleRoot() {
//---------------------------------------------------------------------------------------------------------
   String out = "<!DOCTYPE html><html><head><title>Hoylymoly</title><meta http-equiv='refresh' content='15':URL='";
    out += server.uri()+"'><style>";
    out += "#myT {font-family: Arial, Helvetica, sans-serif;border-collapse: collapse;<!--width: 100%;-->}";
    out += "#myT td,#myT th {border:1px solid #ddd;padding:8px;}";
    out += "#myT th {padding-top:4px;padding-bottom:4px;text-align:left;background-color:#04AA6D;color:white;}";
    out += "h1,h2,h3,h4,h5,h6{font-family:sans-serif;color:maroon;border-bottom:1px solid rgb(200, 200, 200);}";
    out += "</style></head><body>";

    out += "<h2>Hoylymoly Micro-Inverter "+ String(MIWHAT)+"</h2>";
    out += "<h5>"+(String)getDateStr(getNow())+" "+(String)getTimeStr(getNow())+"    started:"+STARTTIME+"</h5>";



    out += "<table id='myT'>";
    out += "<tr><th>MI P</th><th>-Imp/Exp+</th><th>Limit</th><th>U AC</th><th>Freq</th><th>Temp</th></tr>";
    out += "<tr><td>"+String(PMI) +" W</td><td>"+String(GridPower)+" W</td><td>"+String(LIM)+" W</td><td>"+String(U_AC);
    out += " V</td><td>"+String(F_AC)+" Hz</td><td>"+String(TEMP)+" C</td></tr>";


    out += "</table>";

    out += "<table id='myT'>";
    out += "<tr>";
    //<th>Kanal</th><th>PV1</th>PV2</th>PV3</th>PV4</th></tr>";
      for (byte i = 0; i < (ANZAHL_VALUES); i++)
        out += "<th>" +  String(getChannelName(i)) + "</th>";
    out += "</tr>";

    for (byte pv=0; pv <=3; pv++){
      out += "<tr>";
      for (byte i = 0; i < (ANZAHL_VALUES); i++)
        if (i==0) out += "<td>" +  String(int(VALUES[pv][i])) + "</td>"; //PVnr must not be displayed as a floatnr
        else out += "<td>" +  String(VALUES[pv][i]) + "</td>";
      out += "</tr>";
      }

    out += "</table></body></html>";
    server.send (200, "text/html", out);
    //DEBUG_OUT.println (out);
  }


void handleData () {
//---------------------------------------------------------------------------------------------------------
    String out = "";
    for (byte pv=0; pv < 3; pv++){
      for (int i = 0; i < ANZAHL_VALUES; i++) {
        out += String(getChannelName(i)) + '=' + String (VALUES[pv][i]) + '\n';
      }
      server.send(200, "text/plain", out);
    }
  }


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
  }


void setupWebServer (void) {
//---------------------------------------------------------------------------------------------------------
    server.on("/",        handleRoot);
    server.on("/reboot",  handleReboot);
    server.on("/data",    handleData);
    server.on("/help",    handleHelp);
    //server.onNotFound(handleNotFound);    wegen Spiffs-Dateimanager

    server.begin();

    DEBUG_OUT.println ("[HTTP] installed");
  }

void webserverHandle() {
//---------------------------------------------------------------------------------------------------------
    server.handleClient();
  }


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
  #endif

#endif
