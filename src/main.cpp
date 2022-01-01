#include <Arduino.h>
#include <esp_log.h>
#include <sstream>
#include <queue>
#include <string>
#include "eQ3.h"
#include "WiFi.h"
#include "PubSubClient.h"
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <BLEDevice.h>
#include "secrets.h"

// mqtt
string MQTT_SUB = "/command";
string MQTT_PUB = "/state";
string MQTT_PUB2 = "/task";
string MQTT_PUB3 = "/battery";
string MQTT_PUB4 = "/rssi";

#define CARD_KEY "M001AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"

// ---[Variables]---------------------------------------------------------------
eQ3 *keyble;

bool do_open = false;
bool do_lock = false;
bool do_unlock = false;
bool do_status = false;
bool do_toggle = false;
bool do_pair = false;
bool wifiActive = false;
bool cmdTriggered = false;
unsigned long timeout = 0;
bool statusUpdated = false;
bool waitForAnswer = false;
unsigned long starttime = 0;
int status = 0;
int rssi = 0;
int greenLED = 33;
int redLED = 32;

String mqtt_sub = "";
String mqtt_pub = "";
String mqtt_pub2 = "";
String mqtt_pub3 = "";
String mqtt_pub4 = "";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ---[LED Stuff]------------------------------------------------------------
void initLED(){
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
}
void allLEDOff(){
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
}
void redLEDOn(int duration){
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, HIGH);
  if(duration > 0){
    delay(duration);
    allLEDOff();
  }
}
void greenLEDOn(int duration){
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
  if(duration > 0){
    delay(duration);
    allLEDOff();
  }
}

// ---[MqttCallback]------------------------------------------------------------
void MqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("# Message received: ");
  //pair
  if (payload[0] == '6')
  {
    do_pair = true;
    mqtt_sub = "*** pair ***";
    Serial.println(mqtt_sub);
  }
  //toggle
  if (payload[0] == '5')
  {
    do_toggle = true;
    mqtt_sub = "*** toggle ***";
    Serial.println(mqtt_sub);
  }
  //open
  if (payload[0] == '4')
  {
    do_open = true;
    mqtt_sub = "*** open ***";
    Serial.println(mqtt_sub);
  }
  //lock
  if (payload[0] == '3')
  {
    do_lock = true;
    mqtt_sub = "*** lock ***";
    Serial.println(mqtt_sub);
  }
  //unlock
  if (payload[0] == '2')
  {
    do_unlock = true;
    mqtt_sub = "*** unlock ***";
    Serial.println(mqtt_sub);
  }
  //status
  if (payload[0] == '1')
  {
    do_status = true;
    mqtt_sub = "*** status ***";
    Serial.println(mqtt_sub);
  }
}
// ---[MQTTpublish]-------------------------------------------------------------
void assembleTopicAndSend(string topicPart1, string topicPart2, string message)
{
  string topic = topicPart1 + topicPart2;
  mqttClient.publish(topic.c_str(), message.c_str());
  Serial.print("# published ");
  Serial.print(topic.c_str());
  Serial.print("/");
  Serial.println(message.c_str());
  greenLEDOn(300);
}
void MqttPublish()
{
  statusUpdated = false;
  //MQTT_PUB status
  status = keyble->_LockStatus;
  String str_status = "";
  char charBuffer1[9];
  if (status == 1)
    str_status = "moving";
  else if (status == 2)
    str_status = "unlocked";
  else if (status == 3)
    str_status = "locked";
  else if (status == 4)
    str_status = "open";
  else if (status == 9)
    str_status = "timeout";
  else
    str_status = "unknown";
  String strBuffer = String(str_status);
  strBuffer.toCharArray(charBuffer1, 9);
  assembleTopicAndSend(MqttTopic, MQTT_PUB, charBuffer1);
  mqtt_pub = charBuffer1;

  delay(100);

  //MQTT_PUB2 task
  String str_task = "waiting";
  char charBuffer2[8];
  str_task.toCharArray(charBuffer2, 8);
  assembleTopicAndSend(MqttTopic, MQTT_PUB2, charBuffer2);
  mqtt_pub2 = charBuffer2;

  //MQTT_PUB3 battery
  if (keyble->raw_data[1] == 0x81)
  {
    assembleTopicAndSend(MqttTopic, MQTT_PUB3, "false");
    mqtt_pub3 = true;
  }
  if (keyble->raw_data[1] == 0x01)
  {
    assembleTopicAndSend(MqttTopic, MQTT_PUB3, "true");
    mqtt_pub3 = true;
  }

  //MQTT_PUB3 rssi
  rssi = keyble->_RSSI;
  char charBuffer3[4];
  String strRSSI = String(rssi);

  strRSSI.toCharArray(charBuffer3, 4);
  assembleTopicAndSend(MqttTopic, MQTT_PUB4, charBuffer3);
  mqtt_pub4 = charBuffer3;
  Serial.println("# waiting for command...");
}
// ---[MQTT-Setup]--------------------------------------------------------------
void SetupMqtt()
{
  while (!mqttClient.connected())
  { // Loop until we're reconnected to the MQTT server
    mqttClient.setServer(MqttServerName.c_str(), MqttPort);
    mqttClient.setCallback(&MqttCallback);
    Serial.println("# Connect to MQTT-Broker... ");
    if (mqttClient.connect(MqttTopic.c_str(), MqttUserName.c_str(), MqttUserPass.c_str()))
    {
      Serial.println("# Connected!");
      std::string topic = std::string(MqttTopic) + std::string(MQTT_SUB);
      mqttClient.subscribe(topic.c_str());
      Serial.print("subscribed to topic: ");
      Serial.println(topic.c_str());
    }
    else
    {
      Serial.print("!!! error, rc=");
      Serial.println(mqttClient.state());
    }
  }
}
// ---[Wifi Signalquality]-----------------------------------------------------
int GetWifiSignalQuality()
{
  float signal = 2 * (WiFi.RSSI() + 100);
  if (signal > 100)
    return 100;
  else
    return signal;
}
// ---[SetWifi]-----------------------------------------------------------------
void SetWifi(bool active)
{
  wifiActive = active;
  if (active)
  {
    WiFi.mode(WIFI_STA);
    Serial.println("# WiFi enabled");
  }
  else
  {
    WiFi.mode(WIFI_OFF);
    Serial.println("# WiFi disabled");
  }
}
// ---[SetupWiFi]---------------------------------------------------------------
void SetupWifi()
{
  WiFi.begin(ssid.c_str(), password.c_str());
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("# WIFI: connected to SSiD: " + WiFi.SSID());
  }
  int maxWait = 100;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("# WIFI: checking SSiD: " + WiFi.SSID());
    delay(500);

    if (maxWait <= 0)
      ESP.restart();
    maxWait--;
  }
  Serial.println("# WIFI: connected!");
  Serial.println("# WIFI: signalquality: " + String(GetWifiSignalQuality()) + "%");
  Serial.println("# WiFi connected to IP: " + WiFi.localIP().toString());
}
// ---[Setup]-------------------------------------------------------------------
void setup()
{
  initLED();
  redLEDOn(0);
  delay(1000);
  Serial.begin(115200);
  Serial.println("---Starting up...---");
  Serial.setDebugOutput(true);
  SetupWifi();
  //MQTT
  SetupMqtt();

  greenLEDOn(500);

  //Bluetooth
  BLEDevice::init("");
  keyble = new eQ3(KeyBleMac, KeyBleUserKey, KeyBleUserId);
  //get lockstatus on boot
  do_status = true;
}
// ---[loop]--------------------------------------------------------------------
void loop()
{
  // Wifi reconnect
  if (wifiActive)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("# WiFi disconnected, reconnect...");
      SetupWifi();
      redLEDOn(0);
    }
    else
    {
      // MQTT connected?
      if (!mqttClient.connected()) {
        redLEDOn(0);
        if (WiFi.status() == WL_CONNECTED)
        {
          Serial.println("# MQTT disconnected, reconnect...");
          SetupMqtt();
          greenLEDOn(500);
          if (statusUpdated)
          {
            MqttPublish();
          }
        }
      } else if (mqttClient.connected()) {
        mqttClient.loop();
      }
    }
  }
  if (do_open || do_lock || do_unlock || do_status || do_toggle || do_pair)
  {
    String str_task = "working";
    char charBuffer4[8];
    str_task.toCharArray(charBuffer4, 8);
    assembleTopicAndSend(MqttTopic, MQTT_PUB2, charBuffer4);
    mqtt_pub2 = charBuffer4;
    delay(200);
    SetWifi(false);
    redLEDOn(0);
    yield();
    waitForAnswer = true;
    keyble->_LockStatus = -1;
    starttime = millis();

    if (do_open)
    {
      Serial.println("*** open ***");
      keyble->open();
      do_open = false;
    }

    if (do_lock)
    {
      Serial.println("*** lock ***");
      keyble->lock();
      do_lock = false;
    }

    if (do_unlock)
    {
      Serial.println("*** unlock ***");
      keyble->unlock();
      do_unlock = false;
    }

    if (do_status)
    {
      Serial.println("*** get state ***");
      keyble->updateInfo();
      do_status = false;
    }

    if (do_toggle)
    {
      Serial.println("*** toggle ***");
      if ((status == 2) || (status == 4))
      {
        keyble->lock();
        do_lock = false;
      }
      if (status == 3)
      {
        keyble->unlock();
        do_unlock = false;
      }
      do_toggle = false;
    }

    if (do_pair)
    {
      Serial.println("*** pair ***");
      //Parse key card data
      std::string cardKey = CARD_KEY;
      if (cardKey.length() == 56)
      {
        std::string pairMac = cardKey.substr(1, 12);

        pairMac = pairMac.substr(0, 2) + ":" + pairMac.substr(2, 2) + ":" + pairMac.substr(4, 2) + ":" + pairMac.substr(6, 2) + ":" + pairMac.substr(8, 2) + ":" + pairMac.substr(10, 2);
        std::string pairKey = cardKey.substr(14, 32);
        std::string pairSerial = cardKey.substr(46, 10);
      }
      else
      {
        Serial.println("# invalid CardKey! Pattern example:");
        Serial.println("  M followed by KeyBLE MAC length 12");
        Serial.println("  K followed by KeyBLE CardKey length 32");
        Serial.println("  Serialnumber");
        Serial.println("  MxxxxxxxxxxxxKxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxSSSSSSSSSS");
      }
      do_pair = false;
    }
  }

  if (waitForAnswer)
  {
    bool timeout = (millis() - starttime > LOCK_TIMEOUT * 2000 + 1000);
    bool finished = false;

    if ((keyble->_LockStatus != -1) || timeout)
    {
      if (keyble->_LockStatus == 1)
      {
        //Serial.println("Lockstatus 1");
        if (timeout)
        {
          finished = true;
          Serial.println("!!! Lockstatus 1 - timeout !!!");
        }
      }
      else if (keyble->_LockStatus == -1)
      {
        //Serial.println("Lockstatus -1");
        if (timeout)
        {
          keyble->_LockStatus = 9; //timeout
          finished = true;
          Serial.println("!!! Lockstatus -1 - timeout !!!");
        }
      }
      else if (keyble->_LockStatus != 1)
      {
        finished = true;
        //Serial.println("Lockstatus != 1");
      }

      if (finished)
      {
        Serial.println("# Done!");
        do
        {
          keyble->bleClient->disconnect();
          delay(100);
        } while (keyble->state.connectionState != DISCONNECTED && !timeout);

        delay(100);
        yield();

        SetWifi(true);

        greenLEDOn(100);
        statusUpdated = true;
        waitForAnswer = false;
      }
    }
  }
}
