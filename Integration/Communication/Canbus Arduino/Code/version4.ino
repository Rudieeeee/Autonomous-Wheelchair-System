#include <Arduino.h>
#include <Wire.h>
#include <Arduino_CAN.h>

#include <Adafruit_BNO08x.h>

// ============================================================
// Pin setup
// ============================================================

const int STANDBY_PIN = 7;

const int leftHallPin = 4;
const int rightHallPin = 3;

// ============================================================
// BNO085 / BNO08x
// ============================================================

#define BNO08X_RESET -1
#define BNO08X_ADDR  0x4A

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t imuSensorValue;

bool bno_ok = false;

// Best for wheelchair/robot yaw:
// gyro + accelerometer fusion, no magnetometer
sh2_SensorId_t imuReportType = SH2_GAME_ROTATION_VECTOR;

// 100 Hz
const long imuReportIntervalUs = 10000;

float lastYawDeg = 0.0f;
float lastPitchDeg = 0.0f;
float lastRollDeg = 0.0f;

// ============================================================
// CAN IDs
// ============================================================

// Real joystick frame read from wheelchair CAN.
const uint32_t READ_JOYSTICK_CAN_ID_1 = 0x82000300;
const uint32_t READ_JOYSTICK_CAN_ID_2 = 0x82000000;

// Joystick frame to inject.
const uint32_t SEND_JOYSTICK_CAN_ID = 0x82000300;

// Error / knockout frame.
const uint32_t ERROR_CAN_ID = 0x8C000300;

// Heartbeat frame.
const uint32_t HEARTBEAT_CAN_ID = 0x83C30F0F;

// Serial frame.
const uint32_t SERIAL_CAN_ID = 0x8000000E;

// ============================================================
// CAN Data
// ============================================================

uint8_t heartbeatData[7] = {
  0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87
};

uint8_t serialCanData[8] = {
  0x15, 0xC0, 0x0D, 0xE7,
  0x00, 0x00, 0x00, 0x00
};

uint8_t errorData[8] = {
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

// ============================================================
// Timing
// ============================================================

const unsigned long SENSOR_OUTPUT_PERIOD_MS = 10;

const unsigned long JOYSTICK_SEND_PERIOD_MS   = 10;
const unsigned long HEARTBEAT_SEND_PERIOD_MS  = 100;
const unsigned long SERIAL_CAN_SEND_PERIOD_MS = 500;

const unsigned long SERIAL_TIMEOUT_MS = 300;
const unsigned long SERIAL_SIGN_TIMEOUT_MS = 300;
const unsigned long REAL_JOY_SIGN_TIMEOUT_MS = 500;

const unsigned long STARTUP_WAIT_MS = 5000;

unsigned long lastSensorOutputMs = 0;
unsigned long lastJoystickSendMs = 0;
unsigned long lastHeartbeatSendMs = 0;
unsigned long lastSerialCanSendMs = 0;

unsigned long lastSerialCommandMs = 0;
unsigned long lastRealJoystickMs = 0;

unsigned long startupWaitStartMs = 0;

// ============================================================
// Encoder state
// ============================================================

volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile int leftDirection = 0;
volatile int rightDirection = 0;

volatile unsigned long lastLeftInterruptTimeUs = 0;
volatile unsigned long lastRightInterruptTimeUs = 0;

const unsigned long debounceTimeUs = 15000;

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

const int deadband = 5;

// ============================================================
// Startup / injection state
// ============================================================

bool canSeen = false;
bool hasReceivedSerialCommand = false;

bool waitingStartupDelay = false;
bool errorSent = false;
bool injectionEnabled = false;

// Laptop protocol state.
// START enables serial output and accepts J,x,y commands.
// STOP disables all output to the laptop and disables injection.
bool laptopSessionActive = false;

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

void updateWheelDirectionsFromXY(int x, int y);

// ============================================================
// Laptop START/STOP protocol helpers
// ============================================================

void safeStatusPrint(const char *text) {
  if (!laptopSessionActive) {
    return;
  }

  Serial.println(text);
}

void resetLaptopControlledState() {
  hasReceivedSerialCommand = false;
  waitingStartupDelay = false;
  errorSent = false;
  injectionEnabled = false;

  serialJoyX = 0;
  serialJoyY = 0;
  joyXByte = 0x00;
  joyYByte = 0x00;

  lastSerialCommandMs = 0;
  startupWaitStartMs = 0;

  updateWheelDirectionsFromXY(0, 0);
}

void handleStartSignal() {
  laptopSessionActive = true;
  resetLaptopControlledState();

  safeStatusPrint("STATUS,start_received");
  safeStatusPrint("STATUS,waiting_for_J_x_y");
  safeStatusPrint("FORMAT,DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg");
}

void handleStopSignal() {
  resetLaptopControlledState();
  laptopSessionActive = false;
}

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

void enableImuReport() {
  if (!bno08x.enableReport(imuReportType, imuReportIntervalUs)) {
    safeStatusPrint("STATUS,bno085_enable_report_failed");
  }
  else {
    safeStatusPrint("STATUS,bno085_report_enabled");
  }
}

float wrapAngle360(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}

void quaternionToEulerDeg(
  float qr,
  float qi,
  float qj,
  float qk,
  float &yawDeg,
  float &pitchDeg,
  float &rollDeg
) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  yawDeg = atan2(
    2.0f * (qi * qj + qk * qr),
    sqi - sqj - sqk + sqr
  ) * RAD_TO_DEG;

  pitchDeg = asin(
    -2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)
  ) * RAD_TO_DEG;

  rollDeg = atan2(
    2.0f * (qj * qk + qi * qr),
    -sqi - sqj + sqk + sqr
  ) * RAD_TO_DEG;

  yawDeg = wrapAngle360(yawDeg);
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

  hasReceivedSerialCommand = true;

  // Start startup delay only after CAN has already been seen.
  if (canSeen && !waitingStartupDelay && !errorSent) {
    waitingStartupDelay = true;
    startupWaitStartMs = millis();

    safeStatusPrint("STATUS,serial_received_startup_wait_5s");
  }

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

    // First step: only detect that CAN exists.
    if (!canSeen) {
      canSeen = true;
      safeStatusPrint("STATUS,can_traffic_detected");
      safeStatusPrint("STATUS,waiting_for_laptop_serial");

      // If laptop serial was already received before CAN,
      // start the 5 second wait now.
      if (hasReceivedSerialCommand && !waitingStartupDelay && !errorSent) {
        waitingStartupDelay = true;
        startupWaitStartMs = millis();

        safeStatusPrint("STATUS,serial_already_received_startup_wait_5s");
      }
    }

    // Real joystick CAN frame for encoder sign fallback.
    // Real joystick CAN frame for encoder sign fallback.
    // Accept either joystick ID for reading only.
    if (
      msg.isExtendedId() &&
      (msg.id == READ_JOYSTICK_CAN_ID_1 || msg.id == READ_JOYSTICK_CAN_ID_2) &&
      msg.data_length >= 2
    ) {
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
  // Protocol commands from Python:
  // START  -> enable laptop session and allow DATA/STATUS output.
  // STOP   -> disable laptop session and send nothing to laptop.
  // J,x,y  -> joystick command, only accepted during START session.

  if (strcmp(line, "START") == 0) {
    handleStartSignal();
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    handleStopSignal();
    return;
  }

  if (!laptopSessionActive) {
    return;
  }

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
  if (!laptopSessionActive) {
    return;
  }

  // Do nothing until CAN traffic has been seen.
  if (!canSeen) {
    return;
  }

  // Do nothing until laptop serial command J,x,y has been received.
  if (!hasReceivedSerialCommand) {
    return;
  }

  // Start 5 second wait once.
  if (!waitingStartupDelay && !errorSent) {
    waitingStartupDelay = true;
    startupWaitStartMs = millis();

    safeStatusPrint("STATUS,startup_wait_5s_started");
    return;
  }

  // After 5 seconds, send error frame once.
  if (waitingStartupDelay && !errorSent) {
    unsigned long nowMs = millis();

    if (nowMs - startupWaitStartMs >= STARTUP_WAIT_MS) {
      CanMsg errorMsg(
        CanExtendedId(ERROR_CAN_ID),
        8,
        errorData
      );

      CAN.write(errorMsg);

      errorSent = true;
      waitingStartupDelay = false;
      injectionEnabled = true;

      unsigned long startMs = millis();
      lastJoystickSendMs = startMs;
      lastHeartbeatSendMs = startMs;
      lastSerialCanSendMs = startMs;

      safeStatusPrint("STATUS,error_sent");
      safeStatusPrint("STATUS,joystick_serial_heartbeat_enabled");
    }
  }
}

// ============================================================
// CAN sending
// ============================================================

void sendJoystickCAN() {
  if (!laptopSessionActive) {
    return;
  }

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

void sendHeartbeatCAN() {
  if (!laptopSessionActive) {
    return;
  }

  if (!injectionEnabled) {
    return;
  }

  unsigned long nowMs = millis();

  if (nowMs - lastHeartbeatSendMs < HEARTBEAT_SEND_PERIOD_MS) {
    return;
  }

  lastHeartbeatSendMs = nowMs;

  CanMsg heartbeatMsg(
    CanExtendedId(HEARTBEAT_CAN_ID),
    7,
    heartbeatData
  );

  CAN.write(heartbeatMsg);
}

void sendSerialCAN() {
  if (!laptopSessionActive) {
    return;
  }

  if (!injectionEnabled) {
    return;
  }

  unsigned long nowMs = millis();

  if (nowMs - lastSerialCanSendMs < SERIAL_CAN_SEND_PERIOD_MS) {
    return;
  }

  lastSerialCanSendMs = nowMs;

  CanMsg serialMsg(
    CanExtendedId(SERIAL_CAN_ID),
    8,
    serialCanData
  );

  CAN.write(serialMsg);
}

// ============================================================
// Sensor output
// ============================================================

void getImuAngles(float &yawDeg, float &pitchDeg, float &rollDeg) {
  if (!bno_ok) {
    yawDeg = 0.0f;
    pitchDeg = 0.0f;
    rollDeg = 0.0f;
    return;
  }

  if (bno08x.wasReset()) {
    safeStatusPrint("STATUS,bno085_reset_detected");
    enableImuReport();
  }

  bool gotNewImuData = false;

  // Read all pending IMU packets.
  // Keep the newest one.
  while (bno08x.getSensorEvent(&imuSensorValue)) {
    float qr, qi, qj, qk;

    if (imuSensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      qr = imuSensorValue.un.gameRotationVector.real;
      qi = imuSensorValue.un.gameRotationVector.i;
      qj = imuSensorValue.un.gameRotationVector.j;
      qk = imuSensorValue.un.gameRotationVector.k;
    }
    else if (imuSensorValue.sensorId == SH2_ROTATION_VECTOR) {
      qr = imuSensorValue.un.rotationVector.real;
      qi = imuSensorValue.un.rotationVector.i;
      qj = imuSensorValue.un.rotationVector.j;
      qk = imuSensorValue.un.rotationVector.k;
    }
    else {
      continue;
    }

    quaternionToEulerDeg(qr, qi, qj, qk, lastYawDeg, lastPitchDeg, lastRollDeg);

    // Match your old BNO055 direction:
    // Old code used yaw = 360 - eul.x()
    lastYawDeg = wrapAngle360(lastYawDeg);

    gotNewImuData = true;
  }

  // If no new packet is available exactly this loop,
  // output the last known valid angle instead of random 0.000.
  yawDeg = lastYawDeg;
  pitchDeg = lastPitchDeg;
  rollDeg = lastRollDeg;
}

void outputData() {
  if (!laptopSessionActive) {
    return;
  }

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

  float yawDeg = 0.0f;
  float pitchDeg = 0.0f;
  float rollDeg = 0.0f;

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
  Serial.print(realJoyY);
  Serial.print(",has_serial=");
  Serial.println(hasReceivedSerialCommand);
  */
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(460800);

  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 3000) {
    delay(10);
  }

  delay(500);

  safeStatusPrint("STATUS,starting");

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

  // Keep 100 kHz first for stability with longer wires.
  // Later you can try 400000 if wiring is short and stable.
  Wire.setClock(100000);

  // ------------------------------------------------------------
  // BNO085 setup
  // If it fails, do NOT stop the program.
  // yaw/pitch/roll will stay 0.000.
  // ------------------------------------------------------------

  if (bno08x.begin_I2C(BNO08X_ADDR, &Wire)) {
    bno_ok = true;
    safeStatusPrint("STATUS,bno085_begin_success");

    delay(100);
    enableImuReport();
  }
  else {
    bno_ok = false;
    safeStatusPrint("STATUS,bno085_begin_failed_using_zero_angles");
  }

  // ------------------------------------------------------------
  // CAN setup
  // ------------------------------------------------------------

  if (!CAN.begin(CanBitRate::BR_125k)) {
    safeStatusPrint("STATUS,can_init_failed");

    while (1) {
      delay(1000);
    }
  }

  safeStatusPrint("STATUS,can_begin_success");
  safeStatusPrint("STATUS,waiting_for_can_traffic");
  safeStatusPrint("FORMAT,DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg");

  setNeutralJoystickOutputOnly();
  updateEncoderDirectionSource();
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  // 1. First keep reading wheelchair CAN.
  readCANBus();

  // 2. Then read ROS/Python serial command:
  // J,x,y
  readSerialJoystick();

  // 3. Startup sequence:
  // CAN seen -> serial received -> wait 5 seconds -> send error once.
  handleStartupSequence();

  // 4. After error:
  // keep sending joystick, serial CAN frame, heartbeat.
  sendJoystickCAN();
  sendSerialCAN();
  sendHeartbeatCAN();

  // 5. Always print sensor data every 10 ms.
  // If BNO failed, yaw/pitch/roll are 0.000.
  outputData();
}