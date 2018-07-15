#if ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncTCP.h>
#include <Hash.h>
#include <FS.h>
#endif
#if ESP32
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#endif
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <LibXSVF.h>

// At boot it will attempt to connect as client.
// If this attempt fails, it will become AP.
// Same ssid/password apply for client and AP.
const char *ssid = "websvf";
const char *password = "12345678";
const char *hostName = "websvf"; // request local name when connected as client
const char *http_username = "admin";
const char *http_password = "admin";


// SKETCH BEGIN
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");

/*
> To change JTAG pinout, edit file
> ~/Arduino/libraries/LibXSVF-ESP8266/src/trunk/xsvftool-esp8266.c
> FPGA  wire     ESP32
> ---   ----     -------
> VCC   brown
> GND   black    GND
> TCK   yellow   18
> TDO   green    19
> TDI   violett  23
> TMS   blue     21
*/
LibXSVF jtag = LibXSVF();

// command response to user typing
String CommandLine(String user)
{
  char jtag_id[100];
  jtag.scan();
  sprintf(jtag_id, "0x%08X", jtag.id());
  #if 0
  if(user == "s")
  {
    jtag.program("/bitstream.svf", 0);
    sprintf(jtag_id, "PROGramming");
  }
  #endif
  return String(jtag_id) + " CommandLine typed: " + user;
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  digitalWrite(LED_BUILTIN, HIGH);
  if(type == WS_EVT_CONNECT){
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
  } else if(type == WS_EVT_ERROR){
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if(type == WS_EVT_PONG){
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");
  } else if(type == WS_EVT_DATA){
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    String msg = "";
    if(info->final && info->index == 0 && info->len == len){
      //the whole message is in a single frame and we got all of it's data
      Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      if(info->opcode == WS_TEXT)
        // client->text("I got your text message in single frame");
        client->text(CommandLine(msg));
      else
        client->binary("I got your binary message");
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(info->index == 0){
        if(info->num == 0)
          Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      if((info->index + len) == info->len){
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if(info->final){
          Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
          if(info->message_opcode == WS_TEXT)
            // client->text("I got your text message in multiple frames");
            client->text(CommandLine(msg));
          else
            client->binary("I got your binary message");
        }
      }
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}


void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  #if ESP8266
  WiFi.hostname(hostName);
  #endif
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(hostName);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("STA: Failed!\n");
    WiFi.disconnect(false);
    delay(1000);
    WiFi.begin(ssid, password);
  }

  //Send OTA events to the browser
  ArduinoOTA.onStart([]() { events.send("Update Start", "ota"); });
  ArduinoOTA.onEnd([]() { events.send("Update End", "ota"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char p[32];
    sprintf(p, "Progress: %u%%\n", (progress/(total/100)));
    events.send(p, "ota");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if(error == OTA_AUTH_ERROR) events.send("Auth Failed", "ota");
    else if(error == OTA_BEGIN_ERROR) events.send("Begin Failed", "ota");
    else if(error == OTA_CONNECT_ERROR) events.send("Connect Failed", "ota");
    else if(error == OTA_RECEIVE_ERROR) events.send("Recieve Failed", "ota");
    else if(error == OTA_END_ERROR) events.send("End Failed", "ota");
  });
  ArduinoOTA.setHostname(hostName);
  ArduinoOTA.begin();

  MDNS.addService("http","tcp",80);

  #if ESP8266
  SPIFFS.begin();
  #endif
  #if ESP32
  SPIFFS.begin(true);
  #endif
  jtag.begin(&SPIFFS);
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!",NULL,millis(),1000);
  });
  server.addHandler(&events);

  #ifdef ESP8266
  server.addHandler(new SPIFFSEditor(http_username,http_password));
  #endif
  #ifdef ESP32
  server.addHandler(new SPIFFSEditor(SPIFFS,http_username,http_password));
  #endif

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

  server.onNotFound([](AsyncWebServerRequest *request){
    Serial.printf("NOT_FOUND: ");
    if(request->method() == HTTP_GET)
      Serial.printf("GET");
    else if(request->method() == HTTP_POST)
      Serial.printf("POST");
    else if(request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if(request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if(request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if(request->method() == HTTP_HEAD)
      Serial.printf("HEAD");
    else if(request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

    if(request->contentLength()){
      Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
      Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
    }

    int headers = request->headers();
    int i;
    for(i=0;i<headers;i++){
      AsyncWebHeader* h = request->getHeader(i);
      Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
    }

    int params = request->params();
    for(i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isFile()){
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      } else if(p->isPost()){
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      } else {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }

    request->send(404);
  });
  server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
    static int packet_counter = 0;
    static int out_of_order = 0;
    static size_t expect_index = 0;
    static char report[256];
    if(!index)
    {
      Serial.printf("UploadStart: %s\n", filename.c_str());
      packet_counter=0;
      expect_index = index; // for out-of-order detection
      out_of_order = 0;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    #if 0
      Serial.printf("%s", (const char*)data);
      if((packet_counter % 100) == 0 || packet_counter < 4 || final != 0)
        Serial.printf("packet %d len=%d\n", packet_counter, len); // the content
      packet_counter++;
    #endif
    if(index != expect_index)
      out_of_order++;
    expect_index = index + len;
    if(out_of_order == 0)
      jtag.play_svf_packet(index, data, len, final, report);
    if(final)
    {
      if(out_of_order != 0)
        request->send(200, "text/plain", "received" + String(out_of_order) + " out-of-order packets");
      else
        request->send(200, "text/plain", report);
      Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
      digitalWrite(LED_BUILTIN, LOW);
    }
  });
  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    if(!index)
      Serial.printf("BodyStart: %u\n", total);
    Serial.printf("%s", (const char*)data);
    if(index + len == total)
      Serial.printf("BodyEnd: %u\n", total);
  });
  server.begin();
}

// Report IP address to serial every 15 seconds.
// On ULX3S board it works only if USB-serial only
// if passthru bitstream is loaded.
// Otherwise try 192.168.4.1, "websvf" hostname
// or tools like arp, nmap, tcpdump, ...
void report_ip()
{
  const int32_t report_interval = 15000; // ms
  static int32_t before_ms;
  int32_t now_ms = millis();
  int32_t diff_ms = now_ms - before_ms;
  if(abs(diff_ms) > report_interval)
  {
    IPAddress ip = WiFi.localIP();
    before_ms = now_ms;
    Serial.println(ip);
  }
}

void loop(){
  // report_ip();
  ArduinoOTA.handle();
}

