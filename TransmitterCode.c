#include <SPI.h>
#include <mcp_can.h>

// Motor control pins
#define ENA 13
#define IN1 12
#define IN2 14

// Encoder pin
#define ENCODER_A 25

// Analog input pins
#define TEMP_PIN 34
#define FUEL_PIN 26

// CAN setup
#define CAN_CS 5
MCP_CAN CAN(CAN_CS);

const float wheel_diameter_cm = 30.0;
const float wheel_circumference_m = 3.1416 * (wheel_diameter_cm / 100.0);
const int pulses_per_revolution = 550;

volatile int encoderCount = 0;
unsigned long prevTime = 0;
bool motorRunning = true;

void IRAM_ATTR readEncoderA() {
  encoderCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoderA, RISING);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);  // Start motor

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("CAN Init Failed");
    delay(500);
  }
  Serial.println("CAN Init Success");
  CAN.setMode(MCP_NORMAL);
}

void loop() {
  // Listen for control commands
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    long unsigned int rxId;
    byte len = 0;
    byte buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    if (rxId == 0x101 && len >= 1) {
      if (buf[0] == 0x00) {
        Serial.println("Received STOP command");
        analogWrite(ENA, 0);
        motorRunning = false;
      } else if (buf[0] == 0x01) {
        Serial.println("Received START command");
        analogWrite(ENA, 255);
        motorRunning = true;
      }
    }
  }

  if (millis() - prevTime >= 1000) {
    prevTime = millis();

    float rpm = 0, speed_kmph = 0;

    if (motorRunning) {
      rpm = (encoderCount * 60.0) / pulses_per_revolution;
      float speed_mps = (rpm * wheel_circumference_m) / 60.0;
      speed_kmph = speed_mps * 3.6;
    }

    int rpm_encoded = (int)(rpm * 10);
    int speed_encoded = (int)(speed_kmph * 10);

    int temp_adc = analogRead(TEMP_PIN);
    float voltage = (temp_adc / 4095.0) * 3.3;
    float temp_c = voltage * 100.0;
    int temp_encoded = (int)(temp_c * 10);

    int fuel_adc = analogRead(FUEL_PIN);
    int fuel_percent = map(fuel_adc, 0, 4095, 0, 100);

    Serial.print("RPM: "); Serial.print(rpm, 1);
    Serial.print("\tSpeed: "); Serial.print(speed_kmph, 1); Serial.print(" km/h");
    Serial.print("\tTemp: "); Serial.print(temp_c, 1); Serial.print(" °C");
    Serial.print("\tFuel: "); Serial.print(fuel_percent); Serial.println(" %");

    // CAN data packet
    byte data[8];
    data[0] = (rpm_encoded >> 8) & 0xFF;
    data[1] = rpm_encoded & 0xFF;
    data[2] = (speed_encoded >> 8) & 0xFF;
    data[3] = speed_encoded & 0xFF;
    data[4] = (temp_encoded >> 8) & 0xFF;
    data[5] = temp_encoded & 0xFF;
    data[6] = fuel_percent;
    data[7] = 0;  // Reserved or unused

    CAN.sendMsgBuf(0x100, 0, 8, data);

    encoderCount = 0;
  }
}
