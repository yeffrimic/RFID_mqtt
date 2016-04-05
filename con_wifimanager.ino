
// Librerias
#include <ESP8266WiFi.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7
#include <TimeLib.h> // TimeTracking
#include <WiFiUdp.h> // UDP packet handling for NTP request
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <SoftwareSerial.h>

//variables globales
SoftwareSerial RFID = SoftwareSerial(13, 12);


//NTP Servers:
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const char* ntpServerName = "time.nist.gov";
boolean NTP = false;
const int timeZone = -6;  // Eastern central Time (USA)

WiFiUDP Udp;
unsigned int localPort = 2390;  // local port to listen for UDP packets

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

//-------- Customise these values -----------
//const char* ssid = "T4M2";
//const char* password = "8E85AE4B7D";

#define ORG "qnu3pg"
#define DEVICE_TYPE "pruebayeffri"
#define DEVICE_ID "RFID-02"
#define TOKEN "ikSw_PzJwlVgbaToN*"
#define TRIGGER_PIN 0 //for reset 
//-------- Customise the above values --------

char server[] = ORG ".messaging.internetofthings.ibmcloud.com";
char authMethod[] = "use-token-auth";
char token[] = TOKEN;
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;

const char publishTopic[] = "iot-2/evt/status/fmt/json";
const char responseTopic[] = "iotdm-1/response";
const char manageTopic[] = "iotdevice-1/mgmt/manage";
const char updateTopic[] = "iotdm-1/device/update";
const char rebootTopic[] = "iotdm-1/mgmt/initiate/device/reboot";

#define SERIALNUMBER "ESP0001"
#define MANUFACTURER "flatbox"
#define MODEL "ESP8266"
#define DEVICECLASS "Wifi"
#define DESCRIPTION "Wifinode"
int FWVERSION = 1;
#define HWVERSION "ESPTOY1.22"
#define DESCRIPTIVELOCATION "CAMPUSTEC"


int verde = 5; // Pin led verde
int rojo = 4;// Pin led rojo
int azul = 2;// Pin led azul


String ISO8601;
String our_id;
String myName = "PruebaRFID2";
String Latitud = "15.30";
String Longitud = "-90.15";

void handleUpdate(byte* payload) {
  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject((char*)payload);
  if (!root.success()) {
    Serial.println("handleUpdate: payload parse FAILED");
    return;
  }
  Serial.println("handleUpdate payload:");
  root.prettyPrintTo(Serial);
  Serial.println();
  JsonObject& d = root["d"];
  JsonArray& fields = d["fields"];
  for (JsonArray::iterator it = fields.begin();
       it != fields.end();
       ++it) {
    JsonObject& field = *it;
    const char* fieldName = field["field"];
    if (strcmp (fieldName, "metadata") == 0) {
      JsonObject& fieldValue = field["value"];
      if (fieldValue.containsKey("name")) {

        const char* nodeID = "";
        nodeID = fieldValue["name"];
        myName = String(nodeID);

        Serial.print("myName:");
        Serial.println(myName);
      }
      if (fieldValue.containsKey("lat")) {

        const char* nodeID = "";
        nodeID = fieldValue["lat"];
        Latitud = String(nodeID);

        Serial.print("Latitud:");
        Serial.println(Latitud);
      }
      if (fieldValue.containsKey("long")) {

        const char* nodeID = "";
        nodeID = fieldValue["long"];
        Longitud = String(nodeID);

        Serial.print("Longitud:");
        Serial.println(Longitud);
      }
    }
    if (strcmp (fieldName, "deviceInfo") == 0) {
      JsonObject& fieldValue = field["value"];
      if (fieldValue.containsKey("fwVersion")) {
        FWVERSION = fieldValue["fwVersion"];
        Serial.print("fwVersion:");
        Serial.println(FWVERSION);
      }
    }
  }
}

void callback(char* topic, byte* payload, unsigned int payloadLength) {
  Serial.print("callback invoked for topic: ");
  Serial.println(topic);
  if (strcmp (responseTopic, topic) == 0) {
    return; // just print of response for now
  }

  if (strcmp (rebootTopic, topic) == 0) {
    Serial.println("Rebooting...");
    ESP.restart();
  }

  if (strcmp (updateTopic, topic) == 0) {
    handleUpdate(payload);
  }
}

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

void mqttConnect() {
  if (!!!client.connected()) {
    Serial.print("Reconnecting MQTT client to ");
    Serial.println(server);
    while (!!!client.connect(clientId, authMethod, token)) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
  }
}

void initManagedDevice() {
  if (client.subscribe("iotdm-1/response")) {
    Serial.println("subscribe to responses OK");
  }
  else {
    Serial.println("subscribe to responses FAILED");
  }

  if (client.subscribe(rebootTopic)) {
    Serial.println("subscribe to reboot OK");
  }
  else {
    Serial.println("subscribe to reboot FAILED");
  }

  if (client.subscribe("iotdm-1/device/update")) {
    Serial.println("subscribe to update OK");
  }
  else {
    Serial.println("subscribe to update FAILED");
  }
  int a = 0;
  if (a == 0) {
    StaticJsonBuffer<1024> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    JsonObject& supports = d.createNestedObject("supports");
    supports["deviceActions"] = true;
    char buff[1024];
    root.printTo(buff, sizeof(buff));
    Serial.println("publishing device metadata:");
    Serial.println(buff);

    if (client.publish(manageTopic, buff)) {
      Serial.println("device Publish ok");
    }

    else {
      Serial.print("device Publish failed:");
    }
    a = 1;
  }
  if (a == 1) {
    StaticJsonBuffer<1024> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    JsonObject& metadata = d.createNestedObject("metadata");
    metadata["name"] = myName;
    metadata["lat"] = Latitud;
    metadata["long"] = Longitud;
    char buff[1024];
    root.printTo(buff, sizeof(buff));
    Serial.println("publishing device metadata:");
    Serial.println(buff);

    if (client.publish(manageTopic, buff)) {
      Serial.println("device Publish ok");
    }

    else {
      Serial.print("device Publish failed:");
    }
    a = 2;
  }
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      NTP = true;
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

void udpConnect() {
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
}

void setup() {
  pinMode(rojo, OUTPUT);
  pinMode(verde, OUTPUT);
  pinMode(azul, OUTPUT);
  digitalWrite(azul, LOW);
  Serial.begin(115200);
  Serial.println();
  pinMode(TRIGGER_PIN, INPUT);
  RFID.begin(9600);
  delay(100);

  while (WiFi.status() != WL_CONNECTED) {
    wifimanager();
    if ( digitalRead(TRIGGER_PIN) == LOW ) {
      digitalWrite(azul, LOW);
      delay(2000);
      ESP.reset();

    }
  }
  delay(100);
  mqttConnect();
  delay(100);
  while (NTP == false) {
    udpConnect ();
    delay(500);
  }
  initManagedDevice();
  delay(100);
}

void ISO8601TimeStampDisplay() {
  // digital clock display of the time
  ISO8601 = String (year(), DEC);
  ISO8601 += "-";
  ISO8601 += month();
  ISO8601 += "-";
  ISO8601 += day();
  ISO8601 += "T";
  ISO8601 += hour();
  ISO8601 += ":";
  ISO8601 += minute();
  ISO8601 += ":";
  ISO8601 += second();
  ISO8601 += "-06:00";
  //Serial.println(ISO8601);
}

time_t prevDisplay = 0; // when the digital clock was displayed

void checkTime () {
  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
      ISO8601TimeStampDisplay();
    }
  }
}

void publishData() {
  if (our_id != "") {
    StaticJsonBuffer<1024> jsonbuffer;
    JsonObject& root = jsonbuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    JsonObject& data = d.createNestedObject("data");
    data["name"] = myName;
    data["id"] = our_id;
    data["timestamp"] = ISO8601;
    char payload[1024];
    root.printTo(payload, sizeof(payload));

    Serial.print("Sending payload: ");
    Serial.println(payload);
    if (client.publish(publishTopic, payload, byte(sizeof(payload)))) {
      Serial.println("Publish OK");
      digitalWrite(rojo, LOW);
      digitalWrite(verde, HIGH);
    }
    else {
      Serial.println("Publish FAILED");
      digitalWrite(rojo, HIGH);
      digitalWrite(verde, LOW);
    }
    delay(1000);
    digitalWrite(rojo, LOW);
    digitalWrite(verde, LOW);
  }
}
boolean rfid() {
  if (our_id.length() > 10) {
    Serial.println("el id es :");
    our_id = our_id.substring(1, 13);
    Serial.println(sizeof(our_id));
    return true;
  } else
  {
    return false;
  }
}

void wifimanager() {

  WiFiManager wifiManager;
  digitalWrite(azul, HIGH);
  Serial.println("empezando");
  if (!  wifiManager.autoConnect("FlatWifi")) {
    Serial.println("error no conecto");
    //wifiManager.resetSettings();
    wifiManager.setTimeout(120);

    if (!wifiManager.startConfigPortal("FlatWifi")) {
      digitalWrite(azul, LOW);
      Serial.println("failed to connect and hit timeout");
      digitalWrite(rojo, HIGH);
      delay(3000);
      digitalWrite(rojo, LOW);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }
  //if you get here you have connected to the WiFi

  digitalWrite(azul, LOW);
  digitalWrite(verde, HIGH);
  delay(3000);
  digitalWrite(verde, LOW);
  Serial.println("connected...yeey :)");

}

void loop() {


  if ( digitalRead(TRIGGER_PIN) == LOW ) {

    digitalWrite(azul, LOW);
    ESP.reset();

  }

  while (RFID.available() > 0)
  {
    digitalWrite(azul, HIGH);
    char character;
    character = RFID.read();
    our_id += character;
    delay(15);
    digitalWrite(azul, LOW);
    delay(15);
  }
  if (rfid()) {
    checkTime();
    publishData();
    our_id = "";
    RFID.flush();
  }
  if (!client.loop()) {
    mqttConnect();
  }
}
