#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>

#define CE_PIN   4
#define CSN_PIN  5

RF24 radio(CE_PIN, CSN_PIN);

SPIClass vspi(VSPI);

const byte address[6] = "00001";

Servo esc1;

const int esc1Pin = 25;   // NOT 18

const int minPulse = 1000;
const int maxPulse = 2000;

struct ControlPacket {
  uint16_t ch0;
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
};

ControlPacket data;

unsigned long lastPacketTime = 0;
const unsigned long failsafeTimeout = 150;

void setup() {
  Serial.begin(115200);

  // Start SPI explicitly
  vspi.begin(18, 19, 23, CSN_PIN); 
  radio.begin(&vspi);

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(108);
  radio.openReadingPipe(0, address);
  radio.startListening();

  esc1.setPeriodHertz(50);
  esc1.attach(esc1Pin, minPulse, maxPulse);
  esc1.writeMicroseconds(minPulse);
}

void loop() {

  if (radio.available()) {
    radio.read(&data, sizeof(data));
    lastPacketTime = millis();

    int throttle = map(data.ch0, 0, 1023, minPulse, maxPulse);
    throttle = constrain(throttle, minPulse, maxPulse);

    esc1.writeMicroseconds(throttle);
  }

  // Failsafe
  if (millis() - lastPacketTime > failsafeTimeout) {
    esc1.writeMicroseconds(minPulse);
  }
}
