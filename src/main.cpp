#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "wifiConfig.h"
#include <ESP8266HTTPClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ElegantOTA.h>

// NEEDS "MQTT.0" on 1883 and "REST-API.0 on 8093
const char *iobrokerIpAddress = "192.168.2.28";
String RestCurrentVersionUrl = "http://" + String(iobrokerIpAddress) + ":8093/v1/state/mqtt.0.WindowSensors.CurrentVersion";
String RestStayOnUrl = "http://" + String(iobrokerIpAddress) + ":8093/v1/state/mqtt.0.WindowSensors.StayOn";
// http://192.168.2.28:8093/v1/state/mqtt.0.WindowSensors.3D3346.batteryVoltage

const int THIS_VERSION = 2;

WiFiClient wifiClient;
HTTPClient httpClient;

ESP8266WebServer server(80);

unsigned long lastMsg = 0;

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char macString[7]; // 6 characters + null terminator

void createStates(const char *ids[], int count, const char *type)
{
  for (int i = 0; i < count; i++)
  {

    String json = "{\"type\":\"state\",\"common\":{"
                  "\"name\":\"" +
                  String(ids[i]) + "\","
                                   "\"type\":\"" +
                  String(type) + "\","
                                 "\"role\":\"state\","
                                 "\"read\":true,"
                                 "\"write\":true"
                                 "},\"native\":{}}";

    // Hier ggf. deine URL-Encoding-Funktion verwenden
    String url = "http://" + String(iobrokerIpAddress) + ":8093/v1/object/mqtt.0.WindowSensors." + String(macString) + "." +
                 String(ids[i]) +
                 "?value=" + json;

    WiFiClient client;
    HTTPClient http;

    http.begin(client, url);
    int code = http.GET();
    Serial.println(String("URL ") + url);
    Serial.printf("%s -> HTTP %d\n", ids[i], code);

    http.end();
  }
}

class RestSendTopic
{
public:
  char url[64];
  RestSendTopic() {}
  RestSendTopic(const char *topic)
  {
    snprintf(url, sizeof(url), "http://%s:8093/set/mqtt.0.WindowSensors.%s.%s?value=", iobrokerIpAddress, macString, topic);
  }
};

RestSendTopic windowStateTopic;
RestSendTopic currentVersionTopic;
RestSendTopic batteryVoltageTopic;
RestSendTopic loggingTopic;
RestSendTopic ipAddressTopic;

const int reedSwitch = 13;
const int powerOff = 16; // set to low to turn off LDO

class RestReceiveTopic
{
public:
  char url[64];
  RestReceiveTopic() {}
  RestReceiveTopic(const char *topic, const char *deviceClass, const char *mac)
  {
    snprintf(url, sizeof(url), "%s/%s/%s/?value=", deviceClass, mac, topic);
  }
};

int GetMqttValueOverRest(String url)
{

  HTTPClient httpClient;
  int val = -1;

  Serial.println("Trying to read from " + url);
  if (httpClient.begin(wifiClient, url))
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

void SetMqttValueOverRest(const RestSendTopic &sendTopic, String value)
{
  HTTPClient httpClient;

  // value als Query-Parameter anhängen
  String fullUrl = String(sendTopic.url) + value;
  Serial.println("Trying to write to " + fullUrl + String(" :") + value + String(" :"));

  if (httpClient.begin(wifiClient, fullUrl))
  {
    int httpResponseCode = httpClient.GET(); // simple-api nutzt auch für "set" ein GET!

    if (httpResponseCode > 0)
    {
      String payload = httpClient.getString();
      Serial.println("Response: " + payload);

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (error)
      {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
        httpClient.end();
        return;
      }

      // simple-api gibt beim /set/ ebenfalls {"val":..., "ack":true, ...} zurück
      bool ack = doc["ack"] | false;
      httpClient.end();
      return;
    }
    else
    {
      Serial.println("Error in HTTP set request:0");
      httpClient.end();
      return;
    }
  }
  else
  {
    Serial.println("Error in HTTP set request (httpClient.begin:false) for" + fullUrl);
    httpClient.end();
    return;
  }
}
void sendMqtt(const RestSendTopic &sendTopic, const String &message)
{
  SetMqttValueOverRest(sendTopic, message.c_str());
}

const char *stringStates[] = {
    "battery",
    "ipAddress",
    "currentVersion"};
const char *numberStates[] = {
    "windowState"};

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

  Serial.println();
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

  server.on("/", []()
            { server.send(200, "text/plain", "Hi! This is ElegantOTA Demo. go to subpage /update"); });

  ElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();

  // Retrieve the MAC address of the device
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(macString, "%02X%02X%02X", mac[3], mac[4], mac[5]);

  createStates(stringStates, 3, "string");
  createStates(numberStates, 1, "number");

  windowStateTopic = RestSendTopic("windowState");
  currentVersionTopic = RestSendTopic("currentVersion");
  loggingTopic = RestSendTopic("log");
  batteryVoltageTopic = RestSendTopic("batteryVoltage");
  ipAddressTopic = RestSendTopic("ipAddress");
}

void Log(String string)
{
  snprintf(msg, MSG_BUFFER_SIZE, "%ld", string);
  SetMqttValueOverRest(loggingTopic, msg);
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
        SetMqttValueOverRest(currentVersionTopic, String(THIS_VERSION));
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

  IPAddress ip = WiFi.localIP();
  char ipStr[16];
  snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

  SetMqttValueOverRest(ipAddressTopic, ipStr);
}

void loop()
{

  server.handleClient();
  ElegantOTA.loop();

  snprintf(msg, MSG_BUFFER_SIZE, "%ld", digitalRead(reedSwitch) == HIGH);
  SetMqttValueOverRest(windowStateTopic, msg);

  float vBatt = (analogRead(A0) * 4.2 * 10 / 1023);
  snprintf(msg, MSG_BUFFER_SIZE, "%ld", vBatt);
  SetMqttValueOverRest(batteryVoltageTopic, String(analogRead(A0) * 4.2 * 10 / 1023));

  delay(2000);

  int releasedVersion = GetMqttValueOverRest(RestCurrentVersionUrl);
  if (releasedVersion > THIS_VERSION)
  {
    Serial.printf("There is an update to %d\n", releasedVersion);
    PerformUpdate(releasedVersion);
  }
  else
  {
    Serial.printf("No update available for, released: %d\n", releasedVersion);
  }
  int stayOn = GetMqttValueOverRest(RestStayOnUrl);
  Serial.printf("stayOn is %d\n", stayOn);
  if (stayOn != 1)
  {
    Serial.println("Switching off");
    digitalWrite(powerOff, LOW); // Switch off supply
  }
  else
  {
    Serial.println("STAYING ON!");
  }
  delay(5000);
  PrintRam();
}
