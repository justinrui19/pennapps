#include <WiFi.h>
#include <ArduinoOTA.h>

// Replace with your Wi-Fi credentials
const char* ssid = "esppenn";
const char* password = "emersonni";

// Motor pins
const int motor1 = 4;
const int motor2 = 13;
const int motor3 = 14;
const int motor4 = 16;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  // OTA Setup
  ArduinoOTA.setHostname("esp32-cam"); // Optional: name shown in Arduino IDE
  ArduinoOTA.begin();
  Serial.println("OTA ready");

  // Setup motor pins
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

  // Turn on motors
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, HIGH);
  digitalWrite(motor3, HIGH);
  digitalWrite(motor4, HIGH);

  delay(5000);  // Run motors for 5 seconds

  // Turn off motors
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);
  digitalWrite(motor3, LOW);
  digitalWrite(motor4, LOW);
}

void loop() {
  ArduinoOTA.handle(); // Required to keep OTA alive
}
