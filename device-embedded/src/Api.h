/*
  This ist the API Plugin for IgelBot.
  Its a prototype for a plugin to the main class of modular.
  The plug in si generig, only the methods handlePOST and handleGET must
  be implemented locally
*/

#ifndef api_h
#define api_h

#include <WiFi101.h>
#include <ArduinoJson.h>
#include <WiFiConfig.h>
#include <ChassisWalking.h>
// Thanks to : https://github.com/apolukhin/html_inside_cpp
//#include <webapp/html_begin.pp> // header from this repo
#include <webapp/index.html>
//#include <webapp/html_end.pp>   // header from this repo

// Declarations for WiFi API
WiFiServer* server;
WiFiClient client;
int status = WL_IDLE_STATUS;

// Declare Functions for API
String handleRequest(String req);
void printHeaders();

//////////////////////////////////////////////////
// Implementation for IgelBot
//ChassisWalking *chassis;
void start();
void stop();
void setJobState(IgelJobState job);
String handlePOST(String url, String content);
String handleGET(String url, String params);
//////////////////////////////////////////////////


// Intitalize Wifi API and Webserver
void setupApi(int8_t cs, int8_t irq, int8_t rst, int8_t en, int port) {
  // Setup the WiFi Connection
  WiFi.setPins(cs, irq, rst, en);
  server = new WiFiServer(port);
  // Connect to WiFi or create one
  int cc = 0;
  while (status != WL_CONNECTED || status == WL_AP_LISTENING) {
    if (cc > 0) {
      Serial.print("Create new Accespoint: ");
      Serial.println(ssid_igel);
      status = WiFi.beginAP(ssid_igel);
       if (status != WL_AP_LISTENING) {
         Serial.println("Failed to create Accesspoint");
       }
       break;
    }
    Serial.print("Attempting to connect to SSID (attempt  ");
    Serial.print(cc);
    Serial.print("): ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, password);
    // Wait 10 seconds for connection:
    delay(1000);
    cc++;
  }

  Serial.println("WiFi connected");
  server->begin();
  Serial.println("Web Server started");
  // Print the IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

// Loop for API
void loopApi() {
  if (!client) {
    client = server->available();
  }
  else {
    if (client.available()) {
      // read request
      String req = client.readString();
      //handle Request
      String response = handleRequest(req);
      // send response
      client.print(response.c_str());
      client.stop();
    }
  }
}

void printJSONHeaders() {
  //ToDo: Set proper Headers here
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/json");
  client.println("");
}

void printHTMLHeaders() {
  //ToDo: Set proper Headers here
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("");
}

// General
String handleRequest(String req) {
  //Serial.print("New Request: ");
  //Serial.println(req);
  int sep = req.indexOf("/");
  String method = req.substring(0,sep-1);
  String fullUrl = req.substring(sep, req.indexOf("HTTP")-1);
  String url = fullUrl.substring(0, req.indexOf("?")-1);
  String params = fullUrl.substring(req.indexOf("?"));
  if (method == "POST") {
    //Serial.println(req);
    String content = req.substring(req.indexOf("\r\n\r\n"));
    //Serial.println(content);
    return handlePOST(url, content);
  } else {
    // for now, treat all other requests like GET requests
    return handleGET(url, params);
  }
}

// Implementation of API handler
String handlePOST(String url, String content) {
  if (url == "/setWalkingPattern") {
    size_t bufferSize = 3*JSON_ARRAY_SIZE(4) + JSON_OBJECT_SIZE(4);
    DynamicJsonBuffer jsonBuffer(bufferSize);
    JsonObject& root = jsonBuffer.parseObject(content);
    chassis->frameIntervall = root["frameIntervall"];
    chassis->frameNumber = root["frameNumber"];
    chassis->servomin = root["servomin"];
    chassis->servomax = root["servomax"];
    // Leg specific Settings
    JsonArray& StartPosistons = root["startPosistons"];
    JsonArray& Amplicifations = root["amplicifations"];
    JsonArray& Speed = root["speed"];
    JsonArray& Trim = root["trim"];
    for (int s = 0; s < 4; s++) {
      chassis->startFrame[s] = StartPosistons[s];
      chassis->legAmp[s] = Amplicifations[s];
      chassis->legSpeed[s] = Speed[s];
      chassis->legTrim[s] = Trim[s];
    }
    chassis->reset();
  }
  return "done";
}

String handleGET(String url, String params) {
  if (url == "/start") {
    start();
    printJSONHeaders();
    return "done";
  }
  if (url == "/stop") {
    stop();
    printJSONHeaders();
    return "done";
  }
  if (url == "/mode/train") {
    setJobState(IGEL_TRAIN);
    printJSONHeaders();
    return "done";
  }
  if (url == "/mode/search") {
    setJobState(IGEL_SEARCH);
    printJSONHeaders();
    return "done";
  }
  if (url == "/favicon.ico") {
    printJSONHeaders();
    return "done";
  }
  //Serial.println("Start HTML");
  printHTMLHeaders();
  //Serial.println(html_index);
  //return "<html><h1>It Works!</h1></html>";
  /*for ( char* it=html_index.begin(); it!=html_index.end(); ++it)
    client.write(*it);*/
  int CHUNK_LEN = 50;
  for (int i = 0; i < html_index.length(); i += CHUNK_LEN)
    client.print(html_index.substring(i, i + CHUNK_LEN));
  return "";
}


#endif
