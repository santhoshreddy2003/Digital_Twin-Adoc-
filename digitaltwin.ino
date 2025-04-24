#include <TinyGPS.h>          // Include TinyGPS library
#include <SoftwareSerial.h>   // Include SoftwareSerial library

// GPS setup
TinyGPS gps;
#define VT_RX 4  // Virtual Terminal RX
#define VT_TX 5  // Virtual Terminal TX
SoftwareSerial VirtualTerminal(VT_RX, VT_TX);

// Ultrasonic sensor setup
int trigPin = 6; // Trigger pin
int echoPin = 7; // Echo pin

// Accelerometer setup
const int xPin = A0; // Potentiometer for X-axis
const int yPin = A1; // Potentiometer for Y-axis
const int zPin = A2; // Potentiometer for Z-axis

// Motor speed setup
int m1 = 13, m2 = 12, en = 9;  // Motor control pins
int pot = A3;                 // Potentiometer pin
int st = 0;                   // Speed value
const float maxRPM = 10000.0; // Maximum motor RPM
const float wheelDiameter = 0.5; // Wheel diameter in meters

void setup() {
  // Initialize Serial and Virtual Terminal
  Serial.begin(9600);
  VirtualTerminal.begin(9600);

  // Motor setup
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(en, OUTPUT);

  // Ultrasonic sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  VirtualTerminal.println("System Initialized");
}

void loop() {
  bool newData = false;
  float latitude = 0.0, longitude = 0.0;

  // GPS data handling
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial.available()) {
      char c = Serial.read();
      if (gps.encode(c)) {
        newData = true;
      }
    }
  }

  if (newData) {
    unsigned long age;
    gps.f_get_position(&latitude, &longitude, &age);
  }

  // Accelerometer simulation
  int xRaw = analogRead(xPin); // Read X-axis value
  int yRaw = analogRead(yPin); // Read Y-axis value
  int zRaw = analogRead(zPin); // Read Z-axis value
  float xVoltage = (xRaw * 5.0) / 1023.0; // Scale to 0.00 to 5.00 V
  float yVoltage = (yRaw * 5.0) / 1023.0;
  float zVoltage = (zRaw * 5.0) / 1023.0;

  // Ultrasonic sensor distance measurement
  long duration;
  float distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2.0) / 29.1; // Calculate distance in cm

  // Motor speed control and calculation
  st = analogRead(pot) / 4;              // Read potentiometer value
  analogWrite(en, st);                   // Write speed to motor
  digitalWrite(m1, HIGH);                // Set motor direction
  digitalWrite(m2, LOW);
  float speedPercentage = st / 255.0;    // Speed as a percentage of max
  float motorRPM = maxRPM * speedPercentage;
  float wheelCircumference = 3.14 * wheelDiameter; // Circumference in meters
  float speedMS = wheelCircumference * (motorRPM / 60); // Speed in m/s
  float speedKMH = speedMS * 3.6;        // Convert to km/h

  // Print all values together
  VirtualTerminal.println("----- Sensor Data -----");
  if (latitude != TinyGPS::GPS_INVALID_F_ANGLE && longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
    VirtualTerminal.print("Latitude: ");
    VirtualTerminal.println(latitude, 6);
    VirtualTerminal.print("Longitude: ");
    VirtualTerminal.println(longitude, 6);
  } else {
    VirtualTerminal.println("GPS Data: Not Available");
  }
  VirtualTerminal.print("X-axis Voltage: "); VirtualTerminal.print(xVoltage, 2); VirtualTerminal.println(" V");
  VirtualTerminal.print("Y-axis Voltage: "); VirtualTerminal.print(yVoltage, 2); VirtualTerminal.println(" V");
  VirtualTerminal.print("Z-axis Voltage: "); VirtualTerminal.print(zVoltage, 2); VirtualTerminal.println(" V");
  VirtualTerminal.print("Distance: "); VirtualTerminal.print(distance); VirtualTerminal.println(" cm");
  VirtualTerminal.print("Motor Speed (RPM): "); VirtualTerminal.println(motorRPM);
  VirtualTerminal.print("Vehicle Speed (km/h): "); VirtualTerminal.println(speedKMH);
  VirtualTerminal.println("-----------------------");

  delay(500); // Delay for readability
}
