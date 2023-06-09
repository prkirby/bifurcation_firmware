#include <Arduino.h>
#include "config.h"

// This example uses an ESP32 Development Board
// to connect to shiftr.io.
//
// You can check on your device after a successful
// connection here: https://www.shiftr.io/try.
//
// by Joël Gähwiler
// https://github.com/256dpi/arduino-mqtt

#include <WiFi.h>
#include <MQTT.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

const char ssid[] = WIFI_NAME;
const char pass[] = WIFI_PASS;
const char mqttBroker[] = MQTT_BROKER;

WiFiClient net;
MQTTClient client;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

unsigned long lastMillis = 0;

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

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, pass);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin(mqttBroker, net);
  client.onMessage(messageReceived);

  connect();
  timeClient.begin();
  timeClient.setTimeOffset(-14400);
}

void loop()
{
  client.loop();
  delay(10); // <- fixes some issues with WiFi stability

  if (!client.connected())
  {
    connect();
  }

  // publish a message roughly every second.
  if (millis() - lastMillis > 10000)
  {
    timeClient.update();

    lastMillis = millis();
    client.publish("/hello", timeClient.getFormattedTime());
  }
}