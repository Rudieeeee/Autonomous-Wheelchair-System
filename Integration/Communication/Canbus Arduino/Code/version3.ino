#include <Arduino.h>
#include <Wire.h>
#include <Arduino_CAN.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ============================================================
// Pin setup
// ============================================================

const int STANDBY_PIN = 7;

const int leftHallPin = 2;
const int rightHallPin = 3;

// ============================================================
// BNO055
// ============================================================

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool bno_ok = false;

// ============================================================
// CAN IDs
// ============================================================

// Real joystick frame read from wheelchair CAN.
// Your old sensor code used this full value with extended flag included.
const uint32_t READ_JOYSTICK_CAN_ID = 0x82000000;

// Joystick frame to inject.
// When sending with CanExtendedId(), use clean 29-bit ID.
const uint32_t SEND_JOYSTICK_CAN_ID = 0x02000000;

// Profile frame.
const uint32_t PROFILE_ID = 0x051;

// Error frame:
// 0C000100#0000000000000000
const uint32_t ERROR_CAN_ID = 0x0C000100;

// Optional speed/profile frame.
// Disabled by default in loop.
const uint32_t STEP_CAN_ID = 0x0A040000;

// ============================================================
// Timing
// ============================================================

const unsigned long SENSOR_OUTPUT_PERIOD_MS = 10;
const unsigned long JOYSTICK_SEND_PERIOD_MS = 10;
const unsigned long STEP_SEND_PERIOD_MS = 684;

const unsigned long SERIAL_TIMEOUT_MS = 300;
const unsigned long SERIAL_SIGN_TIMEOUT_MS = 300;
const unsigned long REAL_JOY_SIGN_TIMEOUT_MS = 500;

const unsigned long PROFILE_TO_ERROR_DELAY_MS = 50;

unsigned long lastSensorOutputMs = 0;
unsigned long lastJoystickSendMs = 0;
unsigned long lastStepSendMs = 0;

unsigned long lastSerialCommandMs = 0;
unsigned long lastRealJoystickMs = 0;

unsigned long profileSentMs = 0;

// ============================================================
// Encoder state
// ============================================================

volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile int leftDirection = 0;
volatile int rightDirection = 0;

volatile unsigned long lastLeftInterruptTimeUs = 0;
volatile unsigned long lastRightInterruptTimeUs = 0;

const unsigned long debounceTimeUs = 10000;

// ============================================================
// Joystick state
// ============================================================

// Serial joystick command from ROS/Python.
// Used for CAN injection and, when fresh, encoder sign.
int serialJoyX = 0;
int serialJoyY = 0;

// Real joystick command read from wheelchair CAN.
// Used as fallback for encoder tick sign.
int realJoyX = 0;
int realJoyY = 0;

// CAN bytes used for injected joystick frame.
uint8_t joyXByte = 0x00;
uint8_t joyYByte = 0x00;

// Optional speed byte.
uint8_t speedByte = 0x00;

const int deadband = 5;

// ============================================================
// Startup / injection state
// ============================================================

bool canSeen = false;
bool profileSent = false;
bool errorSent = false;
bool injectionEnabled = false;

// ============================================================
// Serial parser
// Expected from ROS/Python:
//   J,x,y
// Examples:
//   J,100,0
//   J,0,-100
//   J,0,0
// ============================================================

char serialBuffer[40];
uint8_t serialIndex = 0;

// ============================================================
// Helper functions
// ============================================================

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

uint8_t signedCommandToByte(int value) {
  value = clampInt(value, -100, 100);

  if (value > -deadband && value < deadband) {
    value = 0;
  }

  return (uint8_t)((int8_t)value);
}

int decodeJoystickByte(uint8_t raw) {
  int value = (int8_t)raw;

  value = clampInt(value, -100, 100);

  if (value > -deadband && value < deadband) {
    value = 0;
  }

  return value;
}

// ============================================================
// Encoder direction logic
// ============================================================

void updateWheelDirectionsFromXY(int x, int y) {
  int newLeftDirection = 0;
  int newRightDirection = 0;

  if (y > 0) {
    // Forward
    newLeftDirection = +1;
    newRightDirection = +1;
  }
  else if (y < 0) {
    // Backward
    newLeftDirection = -1;
    newRightDirection = -1;
  }
  else {
    if (x > 0) {
      // Rotate / turn right
      newLeftDirection = +1;
      newRightDirection = -1;
    }
    else if (x < 0) {
      // Rotate / turn left
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

void updateEncoderDirectionSource() {
  unsigned long nowMs = millis();

  bool serialFresh =
    lastSerialCommandMs > 0 &&
    (nowMs - lastSerialCommandMs <= SERIAL_SIGN_TIMEOUT_MS);

  bool realJoystickFresh =
    lastRealJoystickMs > 0 &&
    (nowMs - lastRealJoystickMs <= REAL_JOY_SIGN_TIMEOUT_MS);

  if (serialFresh) {
    // Normal autonomous mode:
    // encoder tick sign follows ROS/Python serial command.
    updateWheelDirectionsFromXY(serialJoyX, serialJoyY);
  }
  else if (realJoystickFresh) {
    // Fallback:
    // if serial stopped, encoder tick sign follows real joystick CAN frame.
    updateWheelDirectionsFromXY(realJoyX, realJoyY);
  }
  else {
    // No valid direction source.
    updateWheelDirectionsFromXY(0, 0);
  }
}

void setSerialJoystickCommand(int x, int y) {
  x = clampInt(x, -100, 100);
  y = clampInt(y, -100, 100);

  if (x > -deadband && x < deadband) x = 0;
  if (y > -deadband && y < deadband) y = 0;

  serialJoyX = x;
  serialJoyY = y;

  joyXByte = signedCommandToByte(serialJoyX);
  joyYByte = signedCommandToByte(serialJoyY);

  lastSerialCommandMs = millis();

  updateEncoderDirectionSource();
}

void setNeutralJoystickOutputOnly() {
  joyXByte = 0x00;
  joyYByte = 0x00;
}

// ============================================================
// Hall interrupts
// ============================================================

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

// ============================================================
// CAN reading
// ============================================================

void readCANBus() {
  while (CAN.available()) {
    CanMsg msg = CAN.read();

    // Any CAN frame means bus is alive.
    if (!canSeen) {
      canSeen = true;
      Serial.println("STATUS,can_traffic_detected");
    }

    // Real joystick CAN frame for encoder sign fallback.
    if (msg.isExtendedId() && msg.id == READ_JOYSTICK_CAN_ID && msg.data_length >= 2) {
      realJoyX = decodeJoystickByte(msg.data[0]);
      realJoyY = decodeJoystickByte(msg.data[1]);

      lastRealJoystickMs = millis();

      updateEncoderDirectionSource();
    }
  }
}

// ============================================================
// Serial reading
// ============================================================

void handleSerialLine(char *line) {
  // Expected:
  // J,x,y
  // Example:
  // J,100,0
  // J,0,-100

  if (line[0] != 'J') {
    return;
  }

  char *p1 = strchr(line, ',');
  if (p1 == nullptr) {
    return;
  }

  char *p2 = strchr(p1 + 1, ',');
  if (p2 == nullptr) {
    return;
  }

  int x = atoi(p1 + 1);
  int y = atoi(p2 + 1);

  setSerialJoystickCommand(x, y);
}

void readSerialJoystick() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        handleSerialLine(serialBuffer);
        serialIndex = 0;
      }
    }
    else {
      if (serialIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      }
      else {
        // Overflow protection
        serialIndex = 0;
      }
    }
  }
}

// ============================================================
// Startup CAN sequence
// ============================================================

void handleStartupSequence() {
  if (!canSeen) {
    return;
  }

  if (!profileSent) {
    uint8_t profileData[4] = {
      0x84, 0x00, 0x00, 0x01
    };

    CanMsg profileMsg(
      CanStandardId(PROFILE_ID),
      4,
      profileData
    );

    CAN.write(profileMsg);

    profileSent = true;
    profileSentMs = millis();

    Serial.println("STATUS,profile_sent");
    return;
  }

  if (profileSent && !errorSent) {
    unsigned long nowMs = millis();

    if (nowMs - profileSentMs >= PROFILE_TO_ERROR_DELAY_MS) {
      uint8_t errorData[8] = {
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
      };

      CanMsg errorMsg(
        CanExtendedId(ERROR_CAN_ID),
        8,
        errorData
      );

      CAN.write(errorMsg);

      errorSent = true;
      injectionEnabled = true;

      lastSerialCommandMs = millis();

      Serial.println("STATUS,error_sent");
      Serial.println("STATUS,joystick_injection_enabled");
    }
  }
}

// ============================================================
// CAN sending
// ============================================================

void sendJoystickCAN() {
  if (!injectionEnabled) {
    return;
  }

  unsigned long nowMs = millis();

  if (nowMs - lastJoystickSendMs < JOYSTICK_SEND_PERIOD_MS) {
    return;
  }

  lastJoystickSendMs = nowMs;

  // Encoder sign:
  // serial command if fresh, otherwise real joystick CAN fallback.
  updateEncoderDirectionSource();

  // Safety:
  // if ROS/Python serial stops, inject neutral movement.
  // This does NOT stop encoder sign fallback from real joystick CAN.
  if (nowMs - lastSerialCommandMs > SERIAL_TIMEOUT_MS) {
    setNeutralJoystickOutputOnly();
  }

  uint8_t dataToSend[2] = {
    joyXByte,
    joyYByte
  };

  CanMsg joystickMsg(
    CanExtendedId(SEND_JOYSTICK_CAN_ID),
    2,
    dataToSend
  );

  CAN.write(joystickMsg);
}

void sendStepCANOptional() {
  if (!injectionEnabled) {
    return;
  }

  unsigned long nowMs = millis();

  if (nowMs - lastStepSendMs < STEP_SEND_PERIOD_MS) {
    return;
  }

  lastStepSendMs = nowMs;

  uint8_t stepData[1] = {
    speedByte
  };

  CanMsg stepMsg(
    CanExtendedId(STEP_CAN_ID),
    1,
    stepData
  );

  CAN.write(stepMsg);
}

// ============================================================
// Sensor output
// ============================================================

void getImuAngles(float &yawDeg, float &pitchDeg, float &rollDeg) {
  if (!bno_ok) {
    yawDeg = 0.0;
    pitchDeg = 0.0;
    rollDeg = 0.0;
    return;
  }

  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  yawDeg = 360.0 - eul.x();

  if (yawDeg >= 360.0) {
    yawDeg -= 360.0;
  }

  pitchDeg = eul.z();
  rollDeg = eul.y();
}

void outputData() {
  unsigned long nowMs = millis();

  if (nowMs - lastSensorOutputMs < SENSOR_OUTPUT_PERIOD_MS) {
    return;
  }

  lastSensorOutputMs = nowMs;

  noInterrupts();
  long leftTicksCopy = leftTicks;
  long rightTicksCopy = rightTicks;
  int leftDirectionCopy = leftDirection;
  int rightDirectionCopy = rightDirection;
  interrupts();

  int leftState = digitalRead(leftHallPin);
  int rightState = digitalRead(rightHallPin);

  float yawDeg = 0.0;
  float pitchDeg = 0.0;
  float rollDeg = 0.0;

  getImuAngles(yawDeg, pitchDeg, rollDeg);

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

  // Optional debug line. Keep commented during ROS use because it may spam serial.
  /*
  Serial.print("DEBUG,dir_left=");
  Serial.print(leftDirectionCopy);
  Serial.print(",dir_right=");
  Serial.print(rightDirectionCopy);
  Serial.print(",serial_x=");
  Serial.print(serialJoyX);
  Serial.print(",serial_y=");
  Serial.print(serialJoyY);
  Serial.print(",real_x=");
  Serial.print(realJoyX);
  Serial.print(",real_y=");
  Serial.println(realJoyY);
  */
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(460800);

  Serial.println("STATUS,starting");

  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);
  delay(10);

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

  Wire.begin();
  Wire.setClock(100000);

  // ------------------------------------------------------------
  // BNO055 setup
  // If it fails, do NOT stop the program.
  // yaw/pitch/roll will stay 0.000.
  // ------------------------------------------------------------

  if (bno.begin(OPERATION_MODE_NDOF)) {
    bno_ok = true;
    Serial.println("STATUS,bno_begin_success");

    delay(1000);
    bno.setExtCrystalUse(true);
  }
  else {
    bno_ok = false;
    Serial.println("STATUS,bno_begin_failed_using_zero_angles");
  }

  // ------------------------------------------------------------
  // CAN setup
  // ------------------------------------------------------------

  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("STATUS,can_init_failed");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,can_begin_success");
  Serial.println("STATUS,waiting_for_can_traffic");
  Serial.println("FORMAT,DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg");

  setNeutralJoystickOutputOnly();
  updateEncoderDirectionSource();
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  // Keep reading wheelchair CAN.
  readCANBus();

  // Keep reading ROS/Python serial command:
  // J,x,y
  readSerialJoystick();

  // After first CAN traffic:
  // send profile once, then error once, then enable injection.
  handleStartupSequence();

  // Send injected joystick command every 10 ms after startup sequence.
  sendJoystickCAN();

  // Optional:
  // Keep disabled first. Enable only if your wheelchair still needs it.
  // sendStepCANOptional();

  // Always print sensor data every 10 ms.
  // If BNO failed, yaw/pitch/roll are 0.000.
  outputData();
}