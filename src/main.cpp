#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "wifiConfig.h"
#include <ESP8266HTTPClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

const char *mqtt_server = "192.168.2.28";
String MqttCurrentVersionUrl = "http://" + String(mqtt_server) + ":8093/v1/state/mqtt.0.WindowSensors.CurrentVersion";
String MqttStayOnUrl = "http://" + String(mqtt_server) + ":8093/v1/state/mqtt.0.WindowSensors.StayOn";
// http://192.168.2.28:8093/v1/state/mqtt.0.WindowSensors.3D3346.batteryVoltage

const int MY_VERSION = 2;

WiFiClient wifiClient;
HTTPClient httpClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastMsg = 0;
char windowStateTopic[50];
char batteryVoltageTopic[50];
char loggingTopic[50];

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char macString[7]; // 6 characters + null terminator

const int reedSwitch = 13;
const int powerOff = 16; // set to low to turn off LDO

void MqttClientConnect()
{
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "esp8266";
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str()))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
int GetMqttValue(String url)
{

  HTTPClient httpClient;
  int val = -1;

  if (httpClient.begin(wifiClient, url))
  {
    Serial.println("Trying to read from " + url);
    int httpResponseCode = httpClient.GET();
    if (httpResponseCode > 0)
    {
      String payload = httpClient.getString();
      Serial.println("Value: " + payload);
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (error)
      {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
        return -1;
      }
      val = doc["val"];
      Serial.print("CurrentVersion from ioBroker: ");
      Serial.println(val);
    }
    else
    {
      Serial.println("Error in HTTP request:0");
    }
    httpClient.end();
    return val;
  }
  else
  {
    Serial.println("Error in HTTP request (httpClient.begin:false)");
    httpClient.end();
    return -1;
  }
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  // initialize the reed switch pin as an input:
  pinMode(reedSwitch, INPUT);
  // initialize the wakeup pin as an input:
  pinMode(powerOff, OUTPUT);
  digitalWrite(powerOff, HIGH);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqtt_server, 1883);
  MqttClientConnect();

  // OTA
  ArduinoOTA.onStart([]()
                     {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type); });

  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  // Retrieve the MAC address of the device
  uint8_t mac[6];
  WiFi.macAddress(mac);
  // Create a string for the last three bytes

  sprintf(macString, "%02X%02X%02X", mac[3], mac[4], mac[5]);
  const char *deviceClassIdentifier = "WindowSensors"; // Example parent device class

  snprintf(windowStateTopic, sizeof(windowStateTopic), "%s/%s/windowState", deviceClassIdentifier, macString);
  snprintf(batteryVoltageTopic, sizeof(windowStateTopic), "%s/%s/batteryVoltage", deviceClassIdentifier, macString);
  snprintf(loggingTopic, sizeof(windowStateTopic), "%s/%s/log", deviceClassIdentifier, macString);
}
void Log(String string)
{
  snprintf(msg, MSG_BUFFER_SIZE, "%ld", string);
  mqttClient.publish(batteryVoltageTopic, msg);
}

void PrintRam()
{
  Serial.print("Free RAM: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
}

void PerformUpdate(int version)
{
  HTTPClient updateHttpClient;

  const char *firmware_url = "http://192.168.2.20:5005/UpdateImages/firmware.bin";

  // Send GET request to the firmware URL
  updateHttpClient.begin(wifiClient, firmware_url);
  updateHttpClient.setAuthorization(WEBDAV_NAME, WEBDAV_PASS);
  int httpCode = updateHttpClient.GET();

  if (httpCode == HTTP_CODE_OK)
  {
    // Check if the server responded with HTTP 200
    int contentLength = updateHttpClient.getSize();
    bool canBegin = Update.begin(contentLength);

    if (canBegin)
    {
      Serial.println(String("Starting firmware update from ") + firmware_url);

      // Stream the firmware binary to the Update library
      WiFiClient &client = updateHttpClient.getStream();
      size_t written = Update.writeStream(client);

      if (written == contentLength)
      {
        Serial.println("Firmware written successfully!");
        Log("Updated to version" + String(version));
      }
      else
      {
        Serial.printf("Firmware write failed. Written: %d, Expected: %d\n", written, contentLength);
      }

      // Finalize the update
      if (Update.end())
      {
        if (Update.isFinished())
        {
          Serial.println("Update completed successfully. Restarting...");
          ESP.restart(); // Restart the device to apply the update
        }
        else
        {
          Serial.println("Update not finished. Something went wrong.");
        }
      }
      else
      {
        Serial.printf("Update failed. Error #: %d\n", Update.getError());
      }
    }
    else
    {
      Serial.println("Not enough space to begin OTA update.");
    }
  }
  else
  {

    Serial.printf("HTTP GET failed. Error code: %d\n", httpCode);
  }

  updateHttpClient.end();
}

void loop()
{

  MqttClientConnect();

  ArduinoOTA.handle();

  snprintf(msg, MSG_BUFFER_SIZE, "%ld", digitalRead(reedSwitch) == HIGH);
  Serial.print("Publish window value: ");
  Serial.print(msg);
  bool success = mqttClient.publish(windowStateTopic, msg, true);
  Serial.println(String(" :") + (success ? " SUCCESS" : "FAIL!"));

  float vBatt = (analogRead(A0) * 4.2 * 10 / 1023);
  snprintf(msg, MSG_BUFFER_SIZE, "%ld", vBatt);
  success = mqttClient.publish(batteryVoltageTopic, msg);
  Serial.print("Publish bat value: ");
  Serial.print(msg);
  Serial.println(String(" :") + (success ? " SUCCESS" : "FAIL!"));

  delay(1000);

  int releasedVersion = GetMqttValue(MqttCurrentVersionUrl);
  if (releasedVersion > MY_VERSION)
  {
    Serial.printf("There is an update to %d\n", releasedVersion);
    PerformUpdate(releasedVersion);
  }
  else
  {
    Serial.printf("No update available for, released: %d\n", releasedVersion);
  }
  int stayOn = GetMqttValue(MqttStayOnUrl);
  if (stayOn = 0)
  {
    Serial.println("Switching off");
  }
  delay(5000);
  digitalWrite(powerOff, LOW); // Switch off supply
  PrintRam();
}