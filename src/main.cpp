#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <ArduinoJson.h>
#include "wifiConfig.h"
#include <ESP8266HTTPClient.h>
#include <Updater.h>

const char *mqtt_server = "192.168.2.28";
String ReadCurrentVersionUrl = "http://" + String(mqtt_server) + ":8093/v1/state/mqtt.0.WindowSensors.CurrentVersion";
// http://192.168.2.28:8093/v1/state/mqtt.0.WindowSensors.3D3346.batteryVoltage

const int MY_VERSION = 1;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char windowStateTopic[50];
char batteryVoltageTopic[50];
char msg[MSG_BUFFER_SIZE];
char macString[7]; // 6 characters + null terminator

uint8_t bridgeAddress1[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}; // please update this with the MAC address of your ESP-NOW TO MQTT brigde
// uint8_t bridgeAddress2[] = {0xNN, 0x4NN, 0xNN, 0xNN, 0xNN, 0xNN};   //please update this with the MAC address of your ESP-NOW TO MQTT brigde

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 20

const int reedSwitch = 13;
const int powerOff = 16; // set to low to turn off LDO

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message
{
  int id;
  char state[7];
  int vBatt;
  float Temp;
} struct_message;

// Create a struct_message called test to store variables to be sent
struct_message myData;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  /* Serial.print("\r\nLast Packet Send Status: ");
    if (sendStatus == 0){
     Serial.println("Delivery success");
    }
    else{
     Serial.println("Delivery fail");
    }
  */
}
void MqttClientConnect()
{
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "esp8288";
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
  int val;
  WiFiClient wifiClient;
  HTTPClient http;
  if (http.begin(wifiClient, ReadCurrentVersionUrl))
  {

    int httpResponseCode = http.GET();
    if (httpResponseCode > 0)
    {
      String payload = http.getString();
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
    http.end();
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
  // WiFi.disconnect();

  delay(1000);
  Serial.print("1");
  delay(1000);
  Serial.print("2");
  delay(1000);
  Serial.print("3");

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

  // Init ESP-NOW
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Set ESP-NOW role
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  // Once ESPNow is successfully init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(bridgeAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  // esp_now_add_peer(bridgeAddress2, ESP_NOW_ROLE_SLAVE, 2, NULL, 0);

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
  const char *firmware_url = "http://yourserver.com/firmware.bin";
  HTTPClient http;

  // Send GET request to the firmware URL
  http.begin(firmware_url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK)
  { // Check if the server responded with HTTP 200
    int contentLength = http.getSize();
    bool canBegin = Update.begin(contentLength);

    if (canBegin)
    {
      Serial.println("Starting firmware update...");

      // Stream the firmware binary to the Update library
      WiFiClient &client = http.getStream();
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

  http.end();
}

void loop()
{
  // Set values to send
  myData.id = BOARD_ID;
  // Read the state of the reed switch and send open or closed
  if (digitalRead(reedSwitch) == HIGH)
  {
    strcpy(myData.state, "Open");
  }
  else
  {
    strcpy(myData.state, "Closed");
  }

  snprintf(msg, MSG_BUFFER_SIZE, "%ld", digitalRead(reedSwitch) == HIGH);
  Serial.print("Publish message: ");
  Serial.println(msg);
  mqttClient.publish(windowStateTopic, msg);

  myData.vBatt = (analogRead(A0) * 4.2 * 10 / 1023);
  snprintf(msg, MSG_BUFFER_SIZE, "%ld", myData.vBatt);
  mqttClient.publish(batteryVoltageTopic, msg);

#ifdef useTempDS18B20
  myData.Temp = sensors.getTempC(Thermometer);
#elseif #elseif useAHT10 || seBME280
  myData.Temp = sensors.getTempC(Thermometer);
  myData.Humid = 0.0; // sensors.getTempC(Thermometer);
#endif

  // Send message via ESP-NOW
  // esp_now_send(0, (uint8_t *)&myData, sizeof(myData));
  // ESP.deepSleep(0);
  delay(10000);

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
  digitalWrite(powerOff, LOW); // Switch off supply
}