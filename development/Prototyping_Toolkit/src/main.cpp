#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include "SparkFun_LIS2DH12.h" //Click here to get the library: http://librarymanager/All#SparkFun_LIS2DH12
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <Preferences.h>

#define I2C_SDA 13
#define I2C_SCL 14

// Flash Memory API for ESP32
// https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
Preferences preferences;
const char preferencesNamespace[] = "BIFURCATION";
unsigned int minPressure;
unsigned int maxPressure;
void initPreferences();

// Wifi Variables
const char ssid[] = WIFI_NAME;
const char pass[] = WIFI_PASS;
const char mqttBroker[] = MQTT_BROKER;
const char hostNamePrefix[] = "BIF_CONT_";
const char boardID[] = BOARD_ID;
const String hostName = String(hostNamePrefix) + String(boardID);
const char otaPass[] = OTA_PASS;
const int otaPort = OTA_PORT;
WiFiClient net;
MQTTClient client;
unsigned long lastMillis = 0;
void mqttConnect();
void initWifi();
void initOta();
void initMqtt();
void otaLoop();
void mqttLoop();
void messageReceived(String &topic, String &payload);

// Accell Variables
SPARKFUN_LIS2DH12 accel; // Create instance
int tapCounter = 0;
float accelX = 0;
float accelY = 0;
float accelZ = 0;
void initAccelerometer();
void checkAccelerometer();

// BMP Variables
Adafruit_BMP280 bmp;
void initPressureSensor();
void checkPressureSensor();
float curTemp = 0;
float curPressure = 0;
float curAltitude = 0;

unsigned long curMillis = 0;
unsigned long prevMillis = 0;
const int statusInterval = 500;
void sendStatus();

// User LED
bool userLedToggled = false;
bool userLedEnabled = false;
const int userLedPin = 2;
void checkUserLed();

// Solenoid and External LED
const int ledPin = 21;
const int sn1CtlPin = 36;
const int sn2CtlPin = 37;
void initLedAndSolenoids();
bool steadyBreath = false;
bool inhaling = false;
bool exhaling = false;
void inhale();
void exhale();
void holdBreath();
void ledOn();
void ledOff();
void breatheSteady();
void setMinPressure(int newMinPressure);
void setMaxPressure(int newMaxPressure);

/**
 * @brief
 *
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(100); // wait for native usb
  Serial.println("Bifurcation Control Stress Test");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);

  initPreferences();

  initWifi();
  initOta();
  initMqtt();

  initAccelerometer();
  initPressureSensor();
  initLedAndSolenoids();

  pinMode(userLedPin, OUTPUT);
  digitalWrite(userLedPin, HIGH);
}

/**
 * @brief
 *
 */
void loop()
{
  otaLoop();
  mqttLoop();
  checkPressureSensor();
  checkAccelerometer();
  checkUserLed();
  curMillis = millis();
  if (curMillis - prevMillis > statusInterval)
  {
    sendStatus();
    prevMillis = curMillis;
  }
  if (steadyBreath)
  {
    breatheSteady();
  }
}

/**
 * @brief
 *
 */
void initPreferences()
{
  preferences.begin(preferencesNamespace, false);
  minPressure = preferences.getUInt("minPressure", 99000); // Default Min Pressure
  maxPressure = preferences.getUInt("maxPressure", 99450); // Default Max Pressure
  Serial.print("Min Pressure From Preferences: ");
  Serial.println(minPressure);
  Serial.print("Max Pressure From Preferences: ");
  Serial.println(maxPressure);
}

/**
 * @brief
 *
 */
void initWifi()
{

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostName.c_str());
  WiFi.begin(ssid, pass);
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/**
 * @brief
 *
 */
void initOta()
{
  /* create a connection at port 3232 */
  ArduinoOTA.setPort(otaPort);
  /* we use mDNS instead of IP of ESP32 directly */
  ArduinoOTA.setHostname(hostName.c_str());

  /* we set password for updating */
  ArduinoOTA.setPassword(otaPass);

  /* this callback function will be invoked when updating start */
  ArduinoOTA.onStart([]()
                     { Serial.println("Start updating"); });
  /* this callback function will be invoked when updating end */
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd updating"); });
  /* this callback function will be invoked when a number of chunks of software was flashed
  so we can use it to calculate the progress of flashing */
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  /* this callback function will be invoked when updating error */
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  /* start updating */
  ArduinoOTA.begin();
}

/**
 * @brief
 *
 */
void initMqtt()
{
  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin(mqttBroker, net);
  client.onMessage(messageReceived);

  mqttConnect();
}

/**
 * @brief
 *
 */
void mqttConnect()
{

  Serial.print("\nconnecting...");
  while (!client.connect("arduino", "public", "public"))
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("/hello");
  client.subscribe("/toggleLed");
  client.subscribe("/inhale");
  client.subscribe("/exhale");
  client.subscribe("/holdBreath");
  client.subscribe("/breatheSteady");
  client.subscribe("/mainLedOn");
  client.subscribe("/mainLedOff");
  client.subscribe("/setMinPressure");
  client.subscribe("/setMaxPressure");
}

/**
 * @brief
 *
 * @param topic
 * @param payload
 */
void messageReceived(String &topic, String &payload)
{
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.

  if (topic == "/toggleLed")
  {
    userLedToggled = true;
    Serial.println("User LED Toggled");
  }
  else if (topic == "/inhale")
  {
    inhale();
    steadyBreath = false;
    Serial.println("Inhaling");
  }
  else if (topic == "/exhale")
  {
    exhale();
    steadyBreath = false;
    Serial.println("Exhaling");
  }
  else if (topic == "/holdBreath")
  {
    holdBreath();
    steadyBreath = false;
    Serial.println("Holding Breath");
  }
  else if (topic == "/breatheSteady")
  {
    steadyBreath = true;
    Serial.println("Breathing Steady");
  }
  else if (topic == "/mainLedOn")
  {
    ledOn();
    Serial.println("Main LED On");
  }
  else if (topic == "/mainLedOff")
  {
    ledOff();
    Serial.println("Main LED Off");
  }
  else if (topic == "/setMinPressure")
  {
    Serial.print("New Min Pressure: ");
    Serial.println(payload);

    setMinPressure(payload.toInt());
  }
  else if (topic == "/setMaxPressure")
  {
    Serial.print("New Max Pressure: ");
    Serial.println(payload);

    setMaxPressure(payload.toInt());
  }
}

/**
 * @brief
 *
 */
void otaLoop()
{
  /* this function will handle incomming chunk of SW, flash and respond sender */
  ArduinoOTA.handle();
}

/**
 * @brief
 *
 */
void mqttLoop()
{
  client.loop();
  delay(5); // <- fixes some issues with WiFi stability

  if (!client.connected())
  {
    mqttConnect();
  }
}

/**
 * @brief
 *
 */
void initAccelerometer()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  while (accel.begin() == false)
  {
    Serial.println("Accelerometer not detected. Are you sure you did a Wire1.begin()? Freezing...");
    ESP.restart();
  }

  // By default, sensor is set to 25Hz, 12bit, 2g at .begin()
  // When these are changed, tap threshold may need to be altered

  accel.enableTapDetection();
  accel.setTapThreshold(40); // 7-bit value. Max value is 127. 10 is a little low. 40 is ok. 100 is a bit hard.

  while (accel.isTapped() == true)
    delay(10); // Clear any initial event that may be in the buffer
  Serial.println("Accelerometer Initialized");
}

/**
 * @brief
 *
 */
void checkAccelerometer()
{
  if (accel.isTapped())
  {
    Serial.print("Tap: ");
    Serial.println(++tapCounter);

    while (accel.isTapped() == true)
      delay(5); // Wait for event to complete
    String tapsMessage = String(tapCounter);
    client.publish("/tapDetected", tapsMessage);
  }

  accelX = accel.getX();
  accelY = accel.getY();
  accelZ = accel.getZ();
}

/**
 * @brief
 *
 */
void initPressureSensor()
{
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // status = bmp.begin();
  if (!status)
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }

  /**
   * Refer to datasheet for appropriate settings:
   * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
   */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X2,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,    /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  Serial.println("Pressure sensor initialized");
}

/**
 * @brief
 *
 */
void checkPressureSensor()
{
  curTemp = bmp.readTemperature();
  curPressure = bmp.readPressure();
  curAltitude = bmp.readAltitude(1013.25);
}

/**
 * @brief
 *
 */
void checkUserLed()
{
  if (!userLedToggled)
    return;

  userLedEnabled = !userLedEnabled;
  userLedToggled = false;

  if (userLedEnabled)
  {
    digitalWrite(userLedPin, LOW);
  }
  else
  {
    digitalWrite(userLedPin, HIGH);
  }
}

/**
 * @brief
 *
 */
void sendStatus()
{
  String pressureMessage = String(curPressure, 4);
  client.publish("/pressure", pressureMessage);
  String tempMessage = String(curTemp, 4);
  client.publish("/temp", tempMessage);
  String altitudeMessage = String(curAltitude);
  client.publish("/altitude", altitudeMessage);
}

/**
 * LED & Solenoid Control
 * IE Breath and Light
 */
void initLedAndSolenoids()
{
  pinMode(sn1CtlPin, OUTPUT);
  pinMode(sn2CtlPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(sn1CtlPin, LOW);
  digitalWrite(sn2CtlPin, LOW);
  digitalWrite(ledPin, LOW);
}

/**
 * @brief
 *
 */
void inhale()
{
  digitalWrite(sn1CtlPin, HIGH);
  digitalWrite(sn2CtlPin, LOW);
  inhaling = true;
  exhaling = false;
}

/**
 * @brief
 *
 */
void exhale()
{
  digitalWrite(sn1CtlPin, LOW);
  digitalWrite(sn2CtlPin, HIGH);
  inhaling = false;
  exhaling = true;
}

/**
 * @brief
 *
 */
void holdBreath()
{
  digitalWrite(sn1CtlPin, LOW);
  digitalWrite(sn2CtlPin, LOW);
  inhaling = false;
  exhaling = true;
}

/**
 * @brief
 *
 */
void breatheSteady()
{
  // Pressure less than min, get to min pressure
  if (curPressure < minPressure && !inhaling)
  {
    inhale();
    return;
  }

  // Pressure greater than maxPressure, handle
  if (curPressure > maxPressure && !exhaling)
  {
    exhale();
    return;
  }

  // If we aren't breathing, start breathing
  if (!inhaling && !exhaling)
  {
    inhale();
    return;
  }
}

/**
 * @brief
 *
 */
void ledOn()
{
  digitalWrite(ledPin, HIGH);
}

/**
 * @brief
 *
 */
void ledOff()
{
  digitalWrite(ledPin, LOW);
}

/**
 * @brief Set the Min Pressure object
 *
 * @param newMinPressure
 */
void setMinPressure(int newMinPressure)
{
  minPressure = newMinPressure;
  preferences.putUInt("minPressure", newMinPressure);

  Serial.print("Min Pressure set to: ");
  Serial.println("minPressure");
}

/**
 * @brief Set the Max Pressure object
 *
 * @param newMaxPressure
 */
void setMaxPressure(int newMaxPressure)
{
  maxPressure = newMaxPressure;
  preferences.putUInt("maxPressure", newMaxPressure);

  Serial.print("Max Pressure set to: ");
  Serial.println(maxPressure);
}