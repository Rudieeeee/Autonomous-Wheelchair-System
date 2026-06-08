#include <Arduino.h>
#include <Wire.h>
#include <Arduino_CAN.h>
#include <Adafruit_BNO08x.h>

// Pins
const int STANDBY_PIN = 7; // Controls CAN transceiver state
const int leftHallPin = 4;
const int rightHallPin = 3;

// BNO085 Setup
#define BNO08X_ADDR  0x4A
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t imuSensorValue;
bool bno_ok = false;

float lastYawDeg = 0.0f, lastPitchDeg = 0.0f, lastRollDeg = 0.0f;
const long imuReportIntervalUs = 10000; // 100 Hz

// CAN IDs for real wheelchair joystick reading
const uint32_t READ_JOYSTICK_CAN_ID_1 = 0x82000300;
const uint32_t READ_JOYSTICK_CAN_ID_2 = 0x82000000;
const int deadband = 5;

// Encoder State
volatile long leftTicks = 0;
volatile long rightTicks = 0;
volatile int leftDirection = 0;
volatile int rightDirection = 0;
volatile unsigned long lastLeftInterruptTimeUs = 0;
volatile unsigned long lastRightInterruptTimeUs = 0;
const unsigned long debounceTimeUs = 15000;

// Timing
const unsigned long SENSOR_OUTPUT_PERIOD_MS = 10; // 100 Hz output
unsigned long lastSensorOutputMs = 0;

// Helper: Decode real wheelchair joystick bytes
int decodeJoystickByte(uint8_t raw) {
  int value = (int8_t)raw;
  if (value > -deadband && value < deadband) value = 0;
  return value;
}

// Convert X/Y joystick into wheel direction states (+1, -1, 0)
void updateWheelDirectionsFromXY(int x, int y) {
  int newLeftDirection = 0, newRightDirection = 0;
  if (y > 0) { newLeftDirection = 1;  newRightDirection = 1; }
  else if (y < 0) { newLeftDirection = -1; newRightDirection = -1; }
  else {
    if (x > 0) { newLeftDirection = 1;  newRightDirection = -1; }
    else if (x < 0) { newLeftDirection = -1; newRightDirection = 1; }
  }
  noInterrupts();
  leftDirection = newLeftDirection;
  rightDirection = newRightDirection;
  interrupts();
}

// Hall Interrupts
void leftHallInterrupt() {
  unsigned long nowUs = micros();
  if (nowUs - lastLeftInterruptTimeUs > debounceTimeUs) {
    leftTicks += leftDirection;
    lastLeftInterruptTimeUs = nowUs;
  }
}

void rightHallInterrupt() {
  unsigned long nowUs = micros();
  if (nowUs - lastRightInterruptTimeUs > debounceTimeUs) {
    rightTicks += rightDirection;
    lastRightInterruptTimeUs = nowUs;
  }
}

void getImuAngles() {
  if (!bno_ok) return;
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, imuReportIntervalUs);
  }

  while (bno08x.getSensorEvent(&imuSensorValue)) {
    if (imuSensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qr = imuSensorValue.un.gameRotationVector.real;
      float qi = imuSensorValue.un.gameRotationVector.i;
      float qj = imuSensorValue.un.gameRotationVector.j;
      float qk = imuSensorValue.un.gameRotationVector.k;

      // Fast Quaternion to Euler Conversion
      float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;
      lastYawDeg = atan2(2.0f * (qi * qj + qk * qr), sqi - sqj - sqk + sqr) * RAD_TO_DEG;
      lastPitchDeg = asin(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
      lastRollDeg = atan2(2.0f * (qj * qk + qi * qr), -sqi - sqj + sqk + sqr) * RAD_TO_DEG;

      if (lastYawDeg < 0.0f) lastYawDeg += 360.0f;
    }
  }
}

void setup() {
  Serial.begin(460800);
  
  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW); // Activate CAN transceiver
  
  pinMode(leftHallPin, INPUT_PULLUP);
  pinMode(rightHallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftHallPin), leftHallInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightHallPin), rightHallInterrupt, RISING);

  Wire.begin();
  Wire.setClock(100000);

  if (bno08x.begin_I2C(BNO08X_ADDR, &Wire)) {
    bno_ok = true;
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, imuReportIntervalUs);
  }

  CAN.begin(CanBitRate::BR_125k);
}

void loop() {
  // 1. Read CAN Bus to update tick directions from user inputs
  while (CAN.available()) {
    CanMsg msg = CAN.read();
    if (msg.isExtendedId() && (msg.id == READ_JOYSTICK_CAN_ID_1 || msg.id == READ_JOYSTICK_CAN_ID_2) && msg.data_length >= 2) {
      int realJoyX = decodeJoystickByte(msg.data[0]);
      int realJoyY = decodeJoystickByte(msg.data[1]);
      updateWheelDirectionsFromXY(realJoyX, realJoyY);
    }
  }

  // 2. Refresh IMU values
  getImuAngles();

  // 3. Stream data packet at 100 Hz
  unsigned long nowMs = millis();
  if (nowMs - lastSensorOutputMs >= SENSOR_OUTPUT_PERIOD_MS) {
    lastSensorOutputMs = nowMs;

    noInterrupts();
    long leftTicksCopy = leftTicks;
    long rightTicksCopy = rightTicks;
    interrupts();

    Serial.print("DATA,");
    Serial.print(nowMs); Serial.print(",");
    Serial.print(leftTicksCopy); Serial.print(",");
    Serial.print(rightTicksCopy); Serial.print(",");
    Serial.print(digitalRead(leftHallPin)); Serial.print(",");
    Serial.print(digitalRead(rightHallPin)); Serial.print(",");
    Serial.print(lastYawDeg, 3); Serial.print(",");
    Serial.print(lastPitchDeg, 3); Serial.print(",");
    Serial.println(lastRollDeg, 3);
  }
}