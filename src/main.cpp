#include <Arduino.h>
// Read analog inputs A0 and A1 on Arduino Nano
// Outputs values to Serial Monitor

const int pinA0 = 32;
const int pinA1 = 33;

void setup() {
  pinMode(pinA0, INPUT);
  pinMode(pinA1, INPUT);
  Serial.begin(115200);  // Faster and cleaner than 9600
}

void loop() {
  int valueA0 = analogRead(pinA0);  // 0–1023
  int valueA1 = analogRead(pinA1);  // 0–1023

  Serial.print("A0: ");
  Serial.print(valueA0);
  Serial.print(" | A1: ");
  Serial.println(valueA1);

  delay(100);  // 10 readings per second
}



/*
#include <ESP32Servo.h>

Servo esc1;
Servo esc2;
Servo esc3;

const int esc1Pin = 18;
const int esc2Pin = 19;
const int esc3Pin = 21;

// Standard ESC pulse range (adjust if needed)
const int minPulse = 1000;  // microseconds
const int maxPulse = 2000;

void setup() {
  esc1.setPeriodHertz(50);  // ESC expects 50Hz
  esc1.attach(esc1Pin, minPulse, maxPulse);

  esc2.setPeriodHertz(50);
  esc2.attach(esc2Pin, minPulse, maxPulse);

  esc3.setPeriodHertz(50);
  esc3.attach(esc3Pin, minPulse, maxPulse);

  // Arm ESC (most require minimum throttle first)
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  delay(4000);
}

void loop() {

  // 25% throttle
  esc1.writeMicroseconds(1250);

  // 50% throttle
  esc2.writeMicroseconds(1500);

  // 100% throttle
  esc3.writeMicroseconds(2000);
  delay(5000);
}
*/