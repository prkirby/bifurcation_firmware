#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_LIS2DH12.h" //Click here to get the library: http://librarymanager/All#SparkFun_LIS2DH12
#include <Adafruit_BMP280.h>

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

unsigned long curMillis = 0;
unsigned long prevMillis = 0;

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
}

void loop()
{
  checkPressureSensor();
  checkAccelerometer();

  curMillis = millis();

  if (curMillis - prevMillis > 1000)
  {
    String accelStatus = "X: " + String(accelX, 4) + " | Y: " + String(accelY, 4) + " | Z: " + String(accelZ, 4);
    Serial.println(accelStatus);

    String pressureStatus = "Temp: " + String(curTemp, 4) + " *C | Press: " + String(curPressure, 4) + " Pa | Alt: " + String(curAltitude, 4) + " m";
    Serial.println(pressureStatus);

    prevMillis = curMillis;
  }
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
}