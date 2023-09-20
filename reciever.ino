/*
  ArduinoMqttClient - WiFi Sender
  SIT210 Embedded System Development
  Task 3.3D
  Name: Samridh Mahajan
  Student Id: 2210994834
*/
#include <ArduinoMqttClient>

// Include the appropriate Wi-Fi library based on the board being used
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

// Network credentials
char wifiSSID[] = "@prabh";             // Your network SSID (name)
char wifiPassword[] = "0987654321";     // Your network password (use for WPA, or use as key for WEP)

int ledPin = 2;

// MQTT setup
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "mqtt-dashboard.com";
int brokerPort = 1883;
const char mqttTopic[] = "SIT210/waves";

void setup() {
  // Initialize serial and wait for port to open
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB port only)
  }

  // Attempt to connect to WiFi network
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(wifiSSID);

  while (WiFi.begin(wifiSSID, wifiPassword) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  // Attempt to connect to the MQTT broker
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, brokerPort)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // Subscribe to a topic
  Serial.print("Subscribing to topic: ");
  Serial.println(mqttTopic);
  Serial.println();

  mqttClient.subscribe(mqttTopic);
}

void loop() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }

    Serial.println();
    Serial.println("LED BLINK");

    // Blink the LED
    for (int i = 0; i < 3; i++) {
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);
    }

    Serial.println();
  }
}
