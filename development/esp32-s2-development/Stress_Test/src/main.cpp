#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include "SparkFun_LIS2DH12.h" //Click here to get the library: http://librarymanager/All#SparkFun_LIS2DH12
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <MQTT.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#define I2C_SDA 13
#define I2C_SCL 14

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

// Wifi Variables
const char ssid[] = WIFI_NAME;
const char pass[] = WIFI_PASS;
const char mqttBroker[] = MQTT_BROKER;
WiFiClient net;
MQTTClient client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
unsigned long lastMillis = 0;
void connect();
void initWifi();
void wifiLoop();

// LED & Solenoid Variables
#define SN1_CTL 36
#define SN2_CTL 37
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
const int ledPin = 21;
const int maxDutyCycle = 255; // 2^8

/**
 * Different modes available:
 *  1 - "Steady"                  {Constant on}
 *  2 - "Blink"                   {Annoying blinking}
 *  3 - "Full Fade - Triangle"    {Fade both LED banks together in pulse - Triangle waveform}
 *  4 - "Full Fade - Saw"         {Fade both LED banks together in pulse - Saw waveform}
 *  5 - "Full Fade - Sine"        {Fade both LED banks together in pulse - Sine waveform}
 *  6 - "Banked Fade - Triangle"  {Fade each LED Banks in and out in pulse - Triangle waveform} NOTE: Doesnt get close to minDIM for some reason
 *  7 - "Banked Fade - Saw"       {Fade each LED Banks in and out in pulse - Saw waveform}
 *  8 - "Banked Fade - Sine"      {Fade each LED Banks in and out in pulse - Sine waveform}
 */
bool status = true;        // on = true | off = false
int mode = 1;              // the current animation mode
int animationTime = 10000; // Milliseconds for a full "loop" of animation
int minDim = 2;            // In 0-255 scale (PWM)
int curMaxDim = 30;        // Current maximum dim, set by potentiometer (in 0-4095 PWM scale)

// 50 hz should be considered as minimum dimming.

// Frequency of the LED lights is 150HZ between two polarities by default
// Frequency of phase shift for each LED "Bank" (Lower freq makes things a bit tripper for steady modes)
// IMPORTANT: frequency seems to be needed to be dialed on a string by string basis
unsigned long curMicros = 0;
unsigned long prevMicros = 0;
unsigned long prevDimMicros = 0;
unsigned long curMillis = 0;
unsigned long prevMillis = 0;
int curAnimTime = 0;
bool phase = true;           // True Bank_A is active, False Bank_B is active
int curDim = 4095;           // Dim between 0 and 255, adjusted by animation functions (in 0-255 PWM scale)
int curDimB = 4095;          // Same as above, but for use in 2 bank patterns only (symmetrical patterns)
bool dir = true;             // true for up, false for down, used in wavetype animations
bool twoBankPattern = false; // Whether or not this is a pattern that switches between each bank
bool bankPhase = false;      // Phase boolean for if we have twoBankPatter (both banks running patterns in symmetry)

bool s1open = false;

double getCurRads();
int sinDimMap(double rads);
int linearDimMap(bool dir);
void initSolenoidsAndLeds();
void swapSolenoids();
void ledAndSolenoidLoop();

void sendStatus();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(100); // wait for native usb
  Serial.println("Bifurcation Control Stress Test");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  initAccelerometer();
  initPressureSensor();
  initWifi();
  initSolenoidsAndLeds();
}

void loop()
{
  wifiLoop();
  checkPressureSensor();
  checkAccelerometer();
  ledAndSolenoidLoop();
}

void initAccelerometer()
{
  while (accel.begin() == false)
  {
    Serial.println("Accelerometer not detected. Are you sure you did a Wire1.begin()? Freezing...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire1.begin(I2C_SDA, I2C_SCL);
    delay(500);
  }

  // By default, sensor is set to 25Hz, 12bit, 2g at .begin()
  // When these are changed, tap threshold may need to be altered

  accel.enableTapDetection();
  accel.setTapThreshold(40); // 7-bit value. Max value is 127. 10 is a little low. 40 is ok. 100 is a bit hard.

  while (accel.isTapped() == true)
    delay(10); // Clear any initial event that may be in the buffer
  Serial.println("Accelerometer Initialized");
}

void checkAccelerometer()
{
  if (accel.isTapped())
  {
    Serial.print("Tap: ");
    Serial.println(++tapCounter);

    while (accel.isTapped() == true)
      delay(5); // Wait for event to complete

    String tapStatus = "Tap Detected!! Number of taps: " + String(tapCounter);
    client.publish("/taps", tapStatus);
  }

  accelX = accel.getX();
  accelY = accel.getY();
  accelZ = accel.getZ();
}

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

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("Pressure sensor initialized");
}

void checkPressureSensor()
{
  curTemp = bmp.readTemperature();
  curPressure = bmp.readPressure();
  curAltitude = bmp.readAltitude(1013.25);
  // Serial.print(F("Temperature = "));
  // Serial.print(bmp.readTemperature());
  // Serial.println(" *C");

  // Serial.print(F("Pressure = "));
  // Serial.print(bmp.readPressure());
  // Serial.println(" Pa");

  // Serial.print(F("Approx altitude = "));
  // Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  // Serial.println(" m");

  // Serial.println();
}

void connect()
{
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("arduino", "public", "public"))
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("/hello");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload)
{
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}

void initWifi()
{
  WiFi.begin(ssid, pass);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin(mqttBroker, net);
  client.onMessage(messageReceived);

  connect();
  timeClient.begin();
  timeClient.setTimeOffset(-14400);
}

void wifiLoop()
{
  client.loop();
  delay(5); // <- fixes some issues with WiFi stability

  if (!client.connected())
  {
    connect();
  }
}

/**
 * @brief Get the Cur Rads based on animation percent
 *
 * @return double
 */
double getCurRads()
{
  double animPercent = double(curAnimTime) / double(animationTime);
  return animPercent * PI;
}

/**
 * @brief Map the diming value based on a sin wav function
 * @return int
 */
int sinDimMap(double rads)
{
  double sinVal = sin(rads);
  if (sinVal < 0)
    sinVal *= -1;
  return map(double(maxDutyCycle * sinVal), 0.0, double(maxDutyCycle), minDim, curMaxDim);
}

/**
 * @brief Map the diming based on animation time linrarly
 * @return int
 */
int linearDimMap(bool dir = true)
{
  int from = dir ? minDim : curMaxDim;
  int to = dir ? curMaxDim : minDim;

  return map(curAnimTime, 0.0, animationTime, from, to);
}

void initSolenoidsAndLeds()
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(SN1_CTL, OUTPUT);
  pinMode(SN2_CTL, OUTPUT);
  digitalWrite(SN1_CTL, LOW);
  digitalWrite(SN2_CTL, HIGH);
}

void swapSolenoids()
{
  if (!s1open)
  {
    digitalWrite(SN1_CTL, HIGH);
    digitalWrite(SN2_CTL, LOW);
    s1open = true;
    client.publish("/solenoids", "SN1 Open | SN2 Closed");
    // Serial.println("SN1 Open | SN2 Closed");
  }
  else
  {
    digitalWrite(SN1_CTL, LOW);
    digitalWrite(SN2_CTL, HIGH);
    s1open = false;
    client.publish("/solenoids", "SN1 Closed | SN2 Open");
    // Serial.println("SN1 Closed | SN2 Open");
  }
}

void ledAndSolenoidLoop()
{
  // Sample time, and set phase for which LED Bank we are currently interested in
  // Check for new dimming value
  curMicros = micros();
  curMillis = millis();

  // Set current anim time based millis
  if (curMillis - prevMillis >= animationTime)
  {
    prevMillis = curMillis;
    dir = !dir;
    swapSolenoids();
    sendStatus(); // Send status after each LED Animation Loop
  }

  curAnimTime = curMillis - prevMillis;

  // Sin Wave
  // curDim = sinDimMap(getCurRads());

  // Triangle Wave
  curDim = linearDimMap(dir);
  Serial.println(curDim);

  analogWrite(ledPin, curDim);
}

void sendStatus()
{
  timeClient.update();

  client.publish("/hello", timeClient.getFormattedTime());

  String accelStatus = "X: " + String(accelX, 4) + " | Y: " + String(accelY, 4) + " | Z: " + String(accelZ, 4);
  client.publish("/accel", accelStatus);

  String pressureStatus = "Temp: " + String(curTemp, 4) + " *C | Press: " + String(curPressure, 4) + " Pa | Alt: " + String(curAltitude, 4) + " m";
  client.publish("/bmp", pressureStatus);
}