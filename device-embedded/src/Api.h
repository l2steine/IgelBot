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
      printHeaders();
      client.print(response.c_str());
      Serial.println(response.c_str());
      client.stop();
    }
  }
}

void printHeaders() {
  //ToDo: Set proper Headers here
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("");
}

// General
String handleRequest(String req) {
  //Serial.println(req);
  int sep = req.indexOf("/");
  String method = req.substring(0,sep-1);
  String fullUrl = req.substring(sep, req.indexOf("HTTP")-1);
  String url = fullUrl.substring(0, req.indexOf("?")-1);
  String params = fullUrl.substring(req.indexOf("?"));
  if (method == "POST") {
    Serial.println(req);
    String content = req.substring(req.indexOf("\r\n\r\n"));
    Serial.println(content);
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

    // Leg specific Settings
    JsonArray& StartPosistons = root["StartPosistons"];
    JsonArray& Amplicifations = root["Amplicifations"];
    JsonArray& Speed = root["Speed"];
    for (int s; s < 4; s++) {
      chassis->legPos[s] = StartPosistons[s];
      chassis->legAmp[s] = Amplicifations[s];
      chassis->legSpeed[s] = Speed[s];
    }
    chassis->reset();
  }
  return "done";
}

String handleGET(String url, String params) {
  if (url == "/start") {
    start();
  }
  if (url == "/stop") {
    stop();
  }
  return "done";
}


#endif
