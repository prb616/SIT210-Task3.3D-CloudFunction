/* SIT-210 (task-3.3D)
   Name : Prabhjot Singh
   Roll No. : 2210994884
  */

// Include necessary libraries and define constants

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

// Define Wi-Fi credentials and sensor pins

char wifiSsid[] = "@prabh";       // Your Wi-Fi network SSID (name)
char wifiPass[] = "0987654321";   // Your Wi-Fi network password

const int trigPin = 2;            // Trigger pin for distance measurement
const int echoPin = 3;            // Echo pin for distance measurement

float pulseDuration, distance;    // Variables for pulse duration and distance

// Initialize Wi-Fi and MQTT clients

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Define MQTT broker information and measurement interval

const char mqttBroker[] = "broker.mqttdashboard.com";
int mqttPort = 1883;
const char mqttTopic[] = "SIT210/waves";

const long measurementInterval = 1000; // Interval for taking distance measurements
unsigned long previousMillis = 0;

int waveCount = 0; // Counter for wave detections

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  while (!Serial);

  // Connect to the Wi-Fi network
  Serial.print("Connecting to a WPA SSID attempt: ");
  Serial.println(wifiSsid);
  while (WiFi.begin(wifiSsid, wifiPass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  // Successfully connected to the Wi-Fi network
  Serial.println("You have a network connection.");
  Serial.println();

  // Connect to the MQTT broker
  Serial.print("Connection attempts to the MQTT broker: ");
  Serial.println(mqttBroker);

  if (!mqttClient.connect(mqttBroker, mqttPort)) {
    // MQTT connection failed
    Serial.print("Connection to MQTT was lost! error number = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  // Successfully connected to the MQTT broker
  Serial.println("You are currently linked to the MQTT broker.");
  Serial.println();
}

void loop() {
  // Poll the MQTT client for incoming messages
  mqttClient.poll();

  // Get the current time in milliseconds
  unsigned long currentMillis = millis();

  // Check if it's time to take a distance measurement
  if (currentMillis - previousMillis >= measurementInterval) {
    previousMillis = currentMillis;

    // Trigger an ultrasonic sensor measurement
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the duration of the pulse
    pulseDuration = pulseIn(echoPin, HIGH);

    // Calculate the distance based on the pulse duration
    distance = (pulseDuration * 0.0343) / 2;

    // Check if a wave is detected (distance less than 5 units)
    if (distance < 5) {
      // Publish a message via MQTT indicating a wave detection
      mqttClient.beginMessage(mqttTopic);
      mqttClient.print("Prabhjot Singh: Wave has been discovered, ");
      mqttClient.print("Distance: ");
      mqttClient.print(distance);
      mqttClient.endMessage();
      delay(1000);
    }

    // Increment the wave detection count
    waveCount++;
  }
}