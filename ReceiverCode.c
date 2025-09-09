#include <Arduino.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <mcp_can.h>

// --------------- Pin Definitions ---------------
#define GPS_RX 4
#define GPS_TX 2
#define SIM900_RX 16
#define SIM900_TX 17
#define CAN_CS 5
#define X_PIN 34
#define Y_PIN 35
#define Z_PIN 32
#define BUTTON_PIN 15  // Button to switch modes

// --------------- GPS & GSM Setup ---------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
HardwareSerial sim900(2);

// --------------- CAN Bus Setup ---------------
MCP_CAN CAN(CAN_CS);
float rpm = 0, speed_kmph = 0, temp_c = 0;
int fuel_percent = 0;

// --------------- SMS Handling ---------------
String incomingSMS = "";
String senderNumber = "";
bool awaitingMessage = false;

// --------------- Accelerometer Setup ---------------
bool messageSent = false;
static unsigned long lastAlertTime = 0;
const unsigned long alertCooldown = 10000; // 10s cooldown between flip alerts

// Phone numbers for car flip alert
const char* phoneNumbers[] = {
  "+919492099635", "+918978693756"
};
const int numRecipients = sizeof(phoneNumbers) / sizeof(phoneNumbers[0]);

// --------------- AT Command Helper ---------------
void sendATCommand(const char *cmd, unsigned long delayMs = 1000) {
  sim900.println(cmd);
  delay(delayMs);
  while (sim900.available()) Serial.write(sim900.read());
  Serial.println();
}

void sendSMS(const char* phoneNumber, const char* message) {
  sendATCommand("AT+CMGF=1", 1000);
  sim900.print("AT+CMGS=\"");
  sim900.print(phoneNumber);
  sim900.println("\"");
  delay(1000);
  sim900.print(message);
  sim900.write(26); // Ctrl+Z
  delay(5000);
  while (sim900.available()) Serial.write(sim900.read());
  Serial.println("SMS sent.");
}

void configureSIM900ForSMS() {
  sendATCommand("AT");
  sendATCommand("AT+CMGF=1");
  sendATCommand("AT+CNMI=2,2,0,0,0");
}

// --------------- Motor Control ---------------
void sendMotorCommand(bool start) {
  byte cmd[1] = { start ? 0x01 : 0x00 };
  CAN.sendMsgBuf(0x101, 0, 1, cmd);
  Serial.println(start ? "Motor START command sent" : "Motor STOP command sent");
}

// --------------- GPS Location Helper ---------------
String getGPSLocation() {
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    return "Lat: " + String(lat, 6) + ", Lng: " + String(lng, 6) +
           "\nMap: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lng, 6);
  } else {
    return "GPS fix not available.";
  }
}

// --------------- Extract Sender Number ---------------
String extractSenderNumber(const String &sms) {
  int idx1 = sms.indexOf('"');
  int idx2 = sms.indexOf('"', idx1 + 1);
  if (idx1 >= 0 && idx2 > idx1) {
    return sms.substring(idx1 + 1, idx2);
  }
  return "";
}

// --------------- Setup ---------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  sim900.begin(9600, SERIAL_8N1, SIM900_RX, SIM900_TX);
  delay(3000);

  configureSIM900ForSMS();

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("CAN Init Failed");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN Init Success");

  Serial.println("Send 'hi', 'start', or 'stop' via SMS to control device.");
}

// --------------- Main Loop ---------------
void loop() {
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  // GPS feed if in GPS mode
  if (!buttonPressed) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
  }

  // Accelerometer Flip Detection only in GPS mode
  if (!buttonPressed) {
    static unsigned long lastAccelCheck = 0;
    if (millis() - lastAccelCheck > 5000) {
      lastAccelCheck = millis();
      int x = analogRead(X_PIN);
      int y = analogRead(Y_PIN);
      int z = analogRead(Z_PIN);
      bool flipped = (z < 1600 || x > 2100);

      if (flipped && (millis() - lastAlertTime > alertCooldown)) {
        String alertMsg = "ALERT: Car Flipped!\nX:" + String(x) + 
                          " Y:" + String(y) + " Z:" + String(z) +
                          "\n" + getGPSLocation();
        for (int i = 0; i < numRecipients; i++) {
          sendSMS(phoneNumbers[i], alertMsg.c_str());
          delay(5000);
        }
        lastAlertTime = millis();
      }
    }
  }

  // Check SIM900 for incoming SMS
  while (sim900.available()) {
    String line = sim900.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) continue;

    Serial.println("SIM900 >> " + line);

    if (line.startsWith("+CMT:")) {
      senderNumber = extractSenderNumber(line);
      awaitingMessage = true;
    }
    else if (awaitingMessage) {
      awaitingMessage = false;
      incomingSMS = line;
      incomingSMS.toLowerCase();
      incomingSMS.trim();

      String reply;

      if (incomingSMS == "hi") {
        if (!buttonPressed) {
          reply = getGPSLocation();  // Only GPS in GPS mode
        } else {
          reply = "RPM: " + String(rpm, 1) +
                  "\nSpeed: " + String(speed_kmph, 1) + " km/h" +
                  "\nTemp: " + String(temp_c, 1) + " °C" +
                  "\nFuel: " + String(fuel_percent) + " %";
        }
      } else if (incomingSMS == "start") {
        sendMotorCommand(true);
        reply = "Motor STARTED.";
      } else if (incomingSMS == "stop") {
        sendMotorCommand(false);
        reply = "Motor STOPPED.";
      } else {
        reply = "Commands:\nhi - GPS or CAN data\nstart - Motor ON\nstop - Motor OFF";
      }

      sendSMS(senderNumber.c_str(), reply.c_str());
    }
  }

  // Manual serial motor test
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') sendMotorCommand(true);
    else if (c == 's') sendMotorCommand(false);
  }

  // CAN Data Receive
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    long unsigned int rxId;
    byte len = 0;
    byte buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    if (rxId == 0x100 && len == 8) {
      int rpm_encoded = (buf[0] << 8) | buf[1];
      int speed_encoded = (buf[2] << 8) | buf[3];
      int temp_encoded = (buf[4] << 8) | buf[5];
      fuel_percent = buf[6];

      rpm = rpm_encoded / 10.0;
      speed_kmph = speed_encoded / 10.0;
      temp_c = temp_encoded / 10.0;

      Serial.print("RPM: "); Serial.print(rpm);
      Serial.print("  Speed: "); Serial.print(speed_kmph); Serial.print(" km/h");
      Serial.print("  Temp: "); Serial.print(temp_c); Serial.print(" °C");
      Serial.print("  Fuel: "); Serial.print(fuel_percent); Serial.println(" %");
    }
  }
}
