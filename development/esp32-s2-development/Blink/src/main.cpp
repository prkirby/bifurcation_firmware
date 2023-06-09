#include <Arduino.h>

/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

#define LED_PIN 2

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop()
{
  digitalWrite(LED_PIN, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(200);                  // wait for a second
  digitalWrite(LED_PIN, LOW);  // turn the LED off by making the voltage LOW
  delay(200);                  // wait for a second
}

// Arduino: 1.8.19 (Mac OS X), Board: "ESP32S2 Dev Module, Disabled, Disabled, Disabled, Disabled, UART0, Disabled, Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS), 240MHz (WiFi), QIO, 80MHz, 4MB (32Mb), 921600, None, Disabled"
//
// Sketch uses 207414 bytes (15%) of program storage space. Maximum is 1310720 bytes.
// Global variables use 15472 bytes (4%) of dynamic memory, leaving 312208 bytes for local variables. Maximum is 327680 bytes.
// esptool.py v4.5.1
// Serial port /dev/cu.usbserial-14230
// Connecting......................................
//
// A fatal error occurred: Failed to connect to ESP32-S2: No serial data received.
// For troubleshooting steps visit: https://docs.espressif.com/projects/esptool/en/latest/troubleshooting.html
// the selected serial port For troubleshooting steps visit: https://docs.espressif.com/projects/esptool/en/latest/troubleshooting.html
//  does not exist or your board is not connected
//
//
// This report would have more information with
//"Show verbose output during compilation"
// option enabled in File -> Preferences.