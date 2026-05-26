#include <Arduino.h>
#include <Wire.h>
#include <Arduino_CAN.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// -------------------- CAN transceiver --------------------
const int STANDBY_PIN = 7;

// -------------------- BNO055 --------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// -------------------- Hall sensor pins --------------------
const int leftHallPin = 2;
const int rightHallPin = 3;

// -------------------- Encoder tick counters --------------------
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
const unsigned long debounceTimeUs = 50000;

// -------------------- CAN joystick --------------------
const uint32_t JOYSTICK_CAN_ID = 0x82000000;

// Joystick values
int joyX = 0;
int joyY = 0;

const int deadband = 5;

// -------------------- Serial output timing --------------------
// 20 ms = 50 Hz. This is good for EKF and SLAM.
const unsigned long outputPeriodMs = 20;
unsigned long lastOutputTimeMs = 0;


// Decode:
// 0x00 = 0
// 0x64 = +100
// 0x9C = -100
int decodeJoystickByte(uint8_t raw) {
  int value = (int8_t)raw;

  if (value > 100) value = 100;
  if (value < -100) value = -100;

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

  if (nowMs - lastOutputTimeMs < outputPeriodMs) {
    return;
  }

  lastOutputTimeMs = nowMs;

  noInterrupts();
  long leftTicksCopy = leftTicks;
  long rightTicksCopy = rightTicks;
  interrupts();

  // BNO055 fused orientation
  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // BNO055 gyro, normally rad/s in Adafruit library
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // BNO055 acceleration, m/s^2
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float yawDeg = eul.x();
  float rollDeg = eul.y();
  float pitchDeg = eul.z();

  float gyroX = gyro.x();
  float gyroY = gyro.y();
  float gyroZ = gyro.z();

  float accelX = accel.x();
  float accelY = accel.y();
  float accelZ = accel.z();

  uint8_t calSys = 0;
  uint8_t calGyro = 0;
  uint8_t calAccel = 0;
  uint8_t calMag = 0;

  bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

  Serial.print("DATA,");
  Serial.print(nowMs);
  Serial.print(",");

  Serial.print(leftTicksCopy);
  Serial.print(",");
  Serial.print(rightTicksCopy);
  Serial.print(",");

  Serial.print(gyroX, 6);
  Serial.print(",");
  Serial.print(gyroY, 6);
  Serial.print(",");
  Serial.print(gyroZ, 6);
  Serial.print(",");

  Serial.print(accelX, 6);
  Serial.print(",");
  Serial.print(accelY, 6);
  Serial.print(",");
  Serial.print(accelZ, 6);
  Serial.print(",");

  Serial.print(yawDeg, 3);
  Serial.print(",");
  Serial.print(pitchDeg, 3);
  Serial.print(",");
  Serial.print(rollDeg, 3);
  Serial.print(",");

  Serial.print(calSys);
  Serial.print(",");
  Serial.print(calGyro);
  Serial.print(",");
  Serial.print(calAccel);
  Serial.print(",");
  Serial.println(calMag);
}


void setup() {
  Serial.begin(460800);

  // Do not block forever if Serial Monitor is not open
  unsigned long startWaitMs = millis();
  while (!Serial && millis() - startWaitMs < 3000) {
    delay(10);
  }

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

  // For wheelchair mapping, IMUPLUS is often better than NDOF
  // because it does not rely on the magnetometer for yaw.
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("STATUS,bno_begin_failed");
    Serial.println("STATUS,check_wiring_or_i2c_address");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,bno_begin_success");
  Serial.println("STATUS,bno_mode_IMUPLUS");

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

  Serial.println(
    "FORMAT,DATA,time_ms,left_ticks,right_ticks,"
    "gyro_x_radps,gyro_y_radps,gyro_z_radps,"
    "accel_x,accel_y,accel_z,"
    "yaw_deg,pitch_deg,roll_deg,"
    "cal_sys,cal_gyro,cal_accel,cal_mag"
  );
}


void loop() {
  readJoystickCAN();
  outputData();
}