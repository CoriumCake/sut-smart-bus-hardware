#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Initialize WiFi in Station mode (no connection needed)
  WiFi.mode(WIFI_STA);
  
  Serial.println("================================");
  Serial.println("   ESP32-CAM MAC Address Finder");
  Serial.println("================================");
  Serial.println();
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println();
  Serial.println("Copy this MAC address and register");
  Serial.println("it on your university's IoT WiFi portal.");
  Serial.println("================================");
}

void loop() {
  // Print every 5 seconds for easy copying
  delay(5000);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}
