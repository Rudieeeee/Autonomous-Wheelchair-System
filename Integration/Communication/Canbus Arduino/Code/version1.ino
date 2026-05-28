#include <Arduino.h>
#include <Wire.h>
#include <Arduino_CAN.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int STANDBY_PIN = 7;

// -------------------- BNO055 --------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// -------------------- Hall sensor pins --------------------
const int leftHallPin = 2;
const int rightHallPin = 3;

// -------------------- Real encoder tick counters --------------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Direction from CAN joystick
// +1 = forward, -1 = backward, 0 = neutral
volatile int leftDirection = 0;
volatile int rightDirection = 0;

// -------------------- Hall debounce --------------------
volatile unsigned long lastLeftInterruptTimeUs = 0;
volatile unsigned long lastRightInterruptTimeUs = 0;

// 50 ms debounce
const unsigned long debounceTimeUs = 10000;

// -------------------- CAN joystick --------------------
const uint32_t JOYSTICK_CAN_ID = 0x82000000;

// Joystick values
int joyX = 0;
int joyY = 0;

const int deadband = 5;

// -------------------- Serial output timing --------------------
const unsigned long outputPeriodMs = 10;   // 100 Hz
unsigned long lastOutputTimeMs = 0;

// Decode:
// 0x00 = 0
// 0x64 = +100
// 0x9C = -100
int decodeJoystickByte(uint8_t raw) {
  int value = (int8_t)raw;

  if (value > 100) {
    value = 100;
  }

  if (value < -100) {
    value = -100;
  }

  if (value > -deadband && value < deadband) {
    value = 0;
  }

  return value;
}

void updateWheelDirectionsFromJoystick() {
  int newLeftDirection = 0;
  int newRightDirection = 0;

  if (joyY > 0) {
    // Forward
    newLeftDirection = +1;
    newRightDirection = +1;
  }
  else if (joyY < 0) {
    // Backward
    newLeftDirection = -1;
    newRightDirection = -1;
  }
  else {
    // Y = 0, rotate around own axis based on X
    if (joyX > 0) {
      // Turn right: left wheel forward, right wheel backward
      newLeftDirection = +1;
      newRightDirection = -1;
    }
    else if (joyX < 0) {
      // Turn left: left wheel backward, right wheel forward
      newLeftDirection = -1;
      newRightDirection = +1;
    }
    else {
      newLeftDirection = 0;
      newRightDirection = 0;
    }
  }

  noInterrupts();
  leftDirection = newLeftDirection;
  rightDirection = newRightDirection;
  interrupts();
}

void readJoystickCAN() {
  if (!CAN.available()) {
    return;
  }

  CanMsg msg = CAN.read();

  if (!msg.isExtendedId()) {
    return;
  }

  if (msg.id != JOYSTICK_CAN_ID) {
    return;
  }

  // Format:
  // DLC = 2
  // data[0] = X
  // data[1] = Y
  if (msg.data_length < 2) {
    return;
  }

  joyX = decodeJoystickByte(msg.data[0]);
  joyY = decodeJoystickByte(msg.data[1]);

  updateWheelDirectionsFromJoystick();
}

// -------------------- Hall interrupts --------------------
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

void outputData() {
  unsigned long nowMs = millis();

  if (nowMs - lastOutputTimeMs >= outputPeriodMs) {
    lastOutputTimeMs = nowMs;

    noInterrupts();
    long leftTicksCopy = leftTicks;
    long rightTicksCopy = rightTicks;
    interrupts();

    int leftState = digitalRead(leftHallPin);
    int rightState = digitalRead(rightHallPin);

    imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    // Original BNO055 fused yaw:
    // float yawDeg = eul.x();

    // Reversed / inverted fused yaw angle
    float yawDeg = 360.0 - eul.x();

    if (yawDeg >= 360.0) {
      yawDeg -= 360.0;
    }

    float rollDeg = eul.y();
    float pitchDeg = eul.z();

    Serial.print("DATA,");
    Serial.print(nowMs);
    Serial.print(",");
    Serial.print(leftTicksCopy);
    Serial.print(",");
    Serial.print(rightTicksCopy);
    Serial.print(",");
    Serial.print(leftState);
    Serial.print(",");
    Serial.print(rightState);
    Serial.print(",");
    Serial.print(yawDeg, 3);
    Serial.print(",");
    Serial.print(pitchDeg, 3);
    Serial.print(",");
    Serial.println(rollDeg, 3);
  }
}

void setup() {
  Serial.begin(460800);
  // while (!Serial);

  Serial.println("STATUS,starting");

  // CAN transceiver standby pin
  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);
  delay(10);

  // Hall sensors
  pinMode(leftHallPin, INPUT_PULLUP);
  pinMode(rightHallPin, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(leftHallPin),
    leftHallInterrupt,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(rightHallPin),
    rightHallInterrupt,
    RISING
  );

  // BNO055 on Arduino GIGA default I2C:
  // SDA = D20, SCL = D21
  Wire.begin();
  Wire.setClock(100000);

  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("STATUS,bno_begin_failed");
    Serial.println("STATUS,check_wiring_or_i2c_address");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,bno_begin_success");

  delay(1000);
  bno.setExtCrystalUse(true);

  // CAN
  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("STATUS,can_init_failed");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,can_begin_success");
  Serial.println("FORMAT,DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg");
}

void loop() {
  readJoystickCAN();
  outputData();
}