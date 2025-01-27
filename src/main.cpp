#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "wifiConfig.h"
#include <ESP8266HTTPClient.h>

const char *mqtt_server = "192.168.2.28";
String ReadCurrentVersionUrl = "http://" + String(mqtt_server) + ":8093/v1/state/mqtt.0.WindowSensors.CurrentVersion";
// http://192.168.2.28:8093/v1/state/mqtt.0.WindowSensors.3D3346.batteryVoltage

const int MY_VERSION = 1;

WiFiClient wifiClient;
HTTPClient httpClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastMsg = 0;
char windowStateTopic[50];
char batteryVoltageTopic[50];

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
      // Once connected, publish an announcement...
      mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttClient.subscribe("inTopic");
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
int GetCurrentVersion()
{
  int val = -1;

  if (httpClient.begin(wifiClient, ReadCurrentVersionUrl))
  {
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
      Serial.println("Error in HTTP request");
    }
    httpClient.end();
    return val;
  }
  else
  {
    Serial.println("Error in HTTP request");
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

  // Retrieve the MAC address of the device
  uint8_t mac[6];
  WiFi.macAddress(mac);
  // Create a string for the last three bytes

  sprintf(macString, "%02X%02X%02X", mac[3], mac[4], mac[5]);
  const char *parentIdentifier = "WindowSensors"; // Example parent device class

  snprintf(windowStateTopic, sizeof(windowStateTopic), "%s/%s/windowState", parentIdentifier, macString);
  snprintf(batteryVoltageTopic, sizeof(windowStateTopic), "%s/%s/batteryVoltage", parentIdentifier, macString);
}
void PerformUpdate()
{
  const char *firmware_url = "http://192.168.2.20:5005/UpdateImages/Update.bin";

  // Send GET request to the firmware URL
  httpClient.begin(wifiClient, firmware_url);
  httpClient.setAuthorization(WEBDAV_NAME, WEBDAV_PASS);
  int httpCode = httpClient.GET();

  if (httpCode == HTTP_CODE_OK)
  { // Check if the server responded with HTTP 200
    int contentLength = httpClient.getSize();
    bool canBegin = Update.begin(contentLength);

    if (canBegin)
    {
      Serial.println("Starting firmware update...");

      // Stream the firmware binary to the Update library
      WiFiClient &client = httpClient.getStream();
      size_t written = Update.writeStream(client);

      if (written == contentLength)
      {
        Serial.println("Firmware written successfully!");
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

  httpClient.end();
}

void loop()
{



  snprintf(msg, MSG_BUFFER_SIZE, "%ld", digitalRead(reedSwitch) == HIGH);
  Serial.print("Publish message: ");
  Serial.println(msg);
  mqttClient.publish(windowStateTopic, msg);

 float vBatt = (analogRead(A0) * 4.2 * 10 / 1023);
  snprintf(msg, MSG_BUFFER_SIZE, "%ld", vBatt);
  mqttClient.publish(batteryVoltageTopic, msg);


  delay(1000);

  int currentVersion = GetCurrentVersion();
  if (currentVersion > MY_VERSION)
  {
    Serial.println("There is an update to " + currentVersion);
    PerformUpdate();
  }
  else
  {
    Serial.println("No update available");
  }
  Serial.println("Switching off");
  delay(1000);
  digitalWrite(powerOff, LOW); // Switch off supply
}