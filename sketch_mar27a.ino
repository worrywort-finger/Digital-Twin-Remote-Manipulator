

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>


const char* ssid = "Oneplus";          // INSERT YOUR NETWORK NAME
const char* password = "123456789"; // INSERT YOUR NETWORK PASSWORD

WiFiUDP udp;
const int udpPort = 4210;
char incomingPacket[255];


Servo panServo;
Servo tiltServo;

// Pin Definitions for Standard ESP32
const int panPin = 19; 
const int tiltPin = 18;

// Kinematic State Variables
int currentPan = 90;
int currentTilt = 90;

void setup() {
  Serial.begin(115200);

  // Initialize Hardware Timers for ESP32 Architecture
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  // Attach and Home Servos
  panServo.attach(panPin, 500, 2400); 
  tiltServo.attach(tiltPin, 500, 2400);
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  // Establish Network Connection
  WiFi.begin(ssid, password);
  Serial.print("Establishing Wi-Fi Connection");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // Output Network Diagnostics
  Serial.println("\n[SYSTEM] Wi-Fi Connection Established.");
  Serial.print("[SYSTEM] Assigned IP Address: ");
  Serial.println(WiFi.localIP()); 
  
  udp.begin(udpPort);
  Serial.printf("[SYSTEM] UDP Listener Active on Port %d\n", udpPort);
}

void loop() {

  int packetSize = udp.parsePacket();
  
  // Execute only if new spatial data has arrived
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0; // Null-terminate the string buffer
    }

    int targetPan, targetTilt;
    
    // Parse the structured string (e.g., "P:100,T:80")
    if (sscanf(incomingPacket, "P:%d,T:%d", &targetPan, &targetTilt) == 2) {
      
      // Safety Boundary Enforcement
      targetPan = constrain(targetPan, 0, 180);
      targetTilt = constrain(targetTilt, 0, 180);

      // Concurrent Hardware Update
      // Both registers are updated synchronously to ensure fluid multi-axis motion
      panServo.write(targetPan);
      tiltServo.write(targetTilt);
    }
  }
}