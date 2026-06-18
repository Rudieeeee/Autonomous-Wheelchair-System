// ============================================================================
// Arduino GIGA combined sketch
// 1) Wheelchair CAN injection + encoders + BNO085 IMU on Arduino_CAN
// 2) ESP ToF CAN master/relay on external MCP2515 using mcp_can
//
// Serial protocol:
//   START        -> enable all laptop/ROS serial output, start ToF live mode
//   STOP         -> disable all laptop/ROS serial output, disable injection, send ToF stop
//   J,x,y        -> joystick command, only accepted after START
//   CAL or C or 2-> start ESP calibration mode, only accepted after START
//   DRIVE or D or 3 -> start ESP difference/live-drive mode, only accepted after START
//   LIVE or 1    -> start ESP live mode, only accepted after START
//   B or BASELINE-> request baseline resend, only accepted after START
//
// Output to laptop is silent until START.
// After START, this sketch prints:
//   DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg
//   TOF64,time_ms,seq,128_hex_chars
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <string.h>
#include <mcp_can.h>
#include <Arduino_CAN.h>
#include <Adafruit_BNO08x.h>

// ============================================================
// External MCP2515 CAN bus for ESP ToF nodes
// ============================================================

const int TOF_SPI_CS_PIN  = 10;
const int TOF_CAN_INT_PIN = 2;

MCP_CAN TOF_CAN(TOF_SPI_CS_PIN);

const uint32_t ID_SYSTEM_CTRL = 0x010;
const uint32_t ID_CALIB_STAT  = 0x012;
const uint32_t ID_MASTER_PING = 0x400;

const uint16_t LIVE_ID[8] = { 0x110, 0x120, 0x130, 0x140, 0x150, 0x160, 0x170, 0x180 };
const uint16_t BASE_ID[8] = { 0x200, 0x202, 0x204, 0x20A, 0x20C, 0x20E, 0x206, 0x208 };
const uint8_t  SENSOR_NODE[8] = { 1, 1, 1, 3, 3, 3, 2, 2 };
const uint8_t  NODE_FIRST[4]  = { 0, 0, 6, 3 };
const uint8_t  NODE_COUNT[4]  = { 0, 3, 2, 3 };

const uint32_t NODE_TIMEOUT_MS  = 2500;
const uint32_t NODE_HW_RESET_MS = 6000;
const int8_t   NODE_RESET_PIN[4] = { -1, -1, -1, -1 };

#define TOF_RX_QUEUE_SIZE 512

struct TofCanMsg {
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];
};

TofCanMsg tof_rx_queue[TOF_RX_QUEUE_SIZE];
volatile uint16_t tof_q_head = 0;
volatile uint16_t tof_q_tail = 0;

uint8_t tof_system_mode = 0;
bool tof_alive[8] = { false };
bool tof_baseline_seen[8] = { false };
uint8_t tof_base_col_mask[8] = { 0 };
uint8_t tof_live_col_mask[8] = { 0 };

uint32_t node_last_seen[4] = { 0 };
uint32_t node_cycle_count[4] = { 0 };
float node_hz[4] = { 0 };
bool node_online[4] = { false };
uint8_t node_reset_reason[4] = { 0 };
uint8_t node_tx_fail_low[4] = { 0 };
uint8_t node_i2c_recover_low[4] = { 0 };
uint8_t node_baseline_ready_mask[4] = { 0 };
uint32_t node_reset_attempt[4] = { 0 };

uint32_t lastTofPing = 0;
uint32_t lastTofHz = 0;
uint32_t lastTofDisplay = 0;
uint32_t lastTofCanCheck = 0;
uint8_t tofPingCount = 0;

uint16_t tof64_words[64];
bool tof64_dirty = false;
uint32_t tof64_seq = 0;
uint32_t lastTof64Publish = 0;
const uint32_t TOF64_PUBLISH_INTERVAL_MS = 66;
const uint16_t NO_DETECTION_MM = 4000;

uint32_t tofRxOverflowTotal = 0;
uint32_t tofSwQueueDropTotal = 0;
uint32_t tofFramesProcessedTotal = 0;

// ============================================================
// Wheelchair / sensor pins
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
sh2_SensorId_t imuReportType = SH2_GAME_ROTATION_VECTOR;
const long imuReportIntervalUs = 10000;  // 100 Hz

float lastYawDeg = 0.0f;
float lastPitchDeg = 0.0f;
float lastRollDeg = 0.0f;

// ============================================================
// Wheelchair CAN IDs
// ============================================================

const uint32_t READ_JOYSTICK_CAN_ID_1 = 0x82000300;
const uint32_t READ_JOYSTICK_CAN_ID_2 = 0x82000000;
const uint32_t SEND_JOYSTICK_CAN_ID   = 0x82000300;
const uint32_t ERROR_CAN_ID           = 0x8C000300;
const uint32_t HEARTBEAT_CAN_ID       = 0x83C30F0F;
const uint32_t SERIAL_CAN_ID          = 0x8000000E;

uint8_t heartbeatData[7] = { 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87 };
uint8_t serialCanData[8] = { 0x15, 0xC0, 0x0D, 0xE7, 0x00, 0x00, 0x00, 0x00 };
uint8_t errorData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

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
// Joystick / startup state
// ============================================================

int serialJoyX = 0;
int serialJoyY = 0;
int realJoyX = 0;
int realJoyY = 0;
uint8_t joyXByte = 0x00;
uint8_t joyYByte = 0x00;
const int deadband = 5;

bool wheelchairCanSeen = false;
bool hasReceivedSerialCommand = false;
bool waitingStartupDelay = false;
bool errorSent = false;
bool injectionEnabled = false;

// Master laptop gate. Nothing is printed to Serial unless this is true.
bool laptopSessionActive = false;

char serialBuffer[48];
uint8_t serialIndex = 0;

void updateWheelDirectionsFromXY(int x, int y);

// ============================================================
// Serial helpers
// ============================================================

void safeStatusPrint(const char *text) {
  if (!laptopSessionActive) return;
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

// ============================================================
// ESP ToF helper functions
// ============================================================

uint16_t packTof64Word(uint16_t distMm, uint8_t colIdx) {
  if (distMm > 8191) distMm = 8191;
  return (uint16_t)((distMm << 3) | (colIdx & 0x07));
}

void initTof64Words() {
  for (uint8_t sensor = 0; sensor < 8; sensor++) {
    for (uint8_t col = 0; col < 8; col++) {
      tof64_words[sensor * 8 + col] = packTof64Word(NO_DETECTION_MM, col);
    }
  }
}

void clearCalibrationTracking() {
  for (uint8_t s = 0; s < 8; s++) {
    tof_baseline_seen[s] = false;
    tof_base_col_mask[s] = 0;
    tof_live_col_mask[s] = 0;
  }
  for (uint8_t n = 1; n <= 3; n++) node_baseline_ready_mask[n] = 0;
}

void sendTofSystemMode(uint8_t targetMode, bool requestBaselineResend = false) {
  tof_system_mode = targetMode;

  byte p[2] = { tof_system_mode, requestBaselineResend ? 0xB1 : 0x00 };
  TOF_CAN.sendMsgBuf(ID_SYSTEM_CTRL, 0, requestBaselineResend ? 2 : 1, p);

  if (tof_system_mode == 2) clearCalibrationTracking();
}

void updateTof64FromCan(char matrixType, uint8_t sensorIdx, const byte buf[8]) {
  if (matrixType != 'L' || sensorIdx >= 8) return;

  for (uint8_t i = 0; i < 4; i++) {
    uint16_t pkt = ((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2];
    uint8_t colIdx = pkt & 0x07;
    uint16_t distMm = (pkt >> 3) & 0x0FFF;
    uint8_t elevationBit = (pkt >> 15) & 1;

    if (elevationBit == 1) distMm = 60;

    if (colIdx < 8) {
      tof64_words[sensorIdx * 8 + colIdx] = packTof64Word(distMm, colIdx);
      tof64_dirty = true;
    }
  }
}

void publishTof64IfDue(uint32_t now) {
  if (!laptopSessionActive) return;
  if (tof_system_mode != 1 && tof_system_mode != 3) return;
  if (!tof64_dirty) return;
  if (now - lastTof64Publish < TOF64_PUBLISH_INTERVAL_MS) return;

  lastTof64Publish = now;
  tof64_dirty = false;
  tof64_seq++;

  Serial.print(F("TOF64,"));
  Serial.print(now);
  Serial.print(F(","));
  Serial.print(tof64_seq);
  Serial.print(F(","));

  char hexWord[5];
  for (uint8_t i = 0; i < 64; i++) {
    snprintf(hexWord, sizeof(hexWord), "%04X", tof64_words[i]);
    Serial.print(hexWord);
  }
  Serial.println();
}

int sensorFromId(uint32_t id, const uint16_t* table) {
  for (int s = 0; s < 8; s++) {
    if (id == table[s] || id == (uint32_t)table[s] + 1) return s;
  }
  return -1;
}

uint8_t halfFromId(uint32_t id, const uint16_t* table, int sensorIdx) {
  if (sensorIdx < 0) return 255;
  return (id == (uint32_t)table[sensorIdx] + 1) ? 1 : 0;
}

void fetchTofCanToRAM() {
  if (digitalRead(TOF_CAN_INT_PIN) == HIGH) return;

  while (TOF_CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rxId;
    uint8_t len;
    uint8_t buf[8];

    if (TOF_CAN.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      uint16_t next_head = (tof_q_head + 1) % TOF_RX_QUEUE_SIZE;
      if (next_head != tof_q_tail) {
        tof_rx_queue[tof_q_head].id = rxId;
        tof_rx_queue[tof_q_head].len = len;
        memcpy(tof_rx_queue[tof_q_head].buf, buf, 8);
        tof_q_head = next_head;
      }
      else {
        tofSwQueueDropTotal++;
      }
    }
  }
}

void handleNodeReply(uint32_t rxId, uint8_t len, const uint8_t buf[8]) {
  uint8_t node = rxId - 0x400;
  if (node < 1 || node > 3) return;

  node_last_seen[node] = millis();

  for (uint8_t k = 0; k < NODE_COUNT[node] && (uint8_t)(1 + k) < len; k++) {
    tof_alive[NODE_FIRST[node] + k] = buf[1 + k];
  }

  if (len >= 5) node_reset_reason[node] = buf[4];
  if (len >= 6) node_tx_fail_low[node] = buf[5];
  if (len >= 7) node_i2c_recover_low[node] = buf[6];
  if (len >= 8) node_baseline_ready_mask[node] = buf[7];

  if (!node_online[node]) {
    node_online[node] = true;
    node_reset_attempt[node] = 0;
  }
}

void processTofRAMQueue() {
  while (tof_q_tail != tof_q_head) {
    uint32_t rxId = tof_rx_queue[tof_q_tail].id;
    uint8_t len = tof_rx_queue[tof_q_tail].len;
    uint8_t buf[8];
    memcpy(buf, tof_rx_queue[tof_q_tail].buf, 8);
    tof_q_tail = (tof_q_tail + 1) % TOF_RX_QUEUE_SIZE;
    tofFramesProcessedTotal++;

    if (rxId >= 0x401 && rxId <= 0x403) {
      handleNodeReply(rxId, len, buf);
    }
    else if (rxId == ID_CALIB_STAT) {
      // Calibration remains on the ESP nodes. Keep this hook for future STATUS lines.
    }
    else {
      int s = sensorFromId(rxId, LIVE_ID);
      if (s >= 0 && len == 8) {
        uint8_t node = SENSOR_NODE[s];
        uint8_t half = halfFromId(rxId, LIVE_ID, s);
        node_last_seen[node] = millis();
        if (rxId == LIVE_ID[NODE_FIRST[node]]) node_cycle_count[node]++;
        if (half < 2) tof_live_col_mask[s] |= (half == 0) ? 0x0F : 0xF0;
        updateTof64FromCan('L', s, buf);
      }
      else {
        s = sensorFromId(rxId, BASE_ID);
        if (s >= 0 && len == 8) {
          uint8_t half = halfFromId(rxId, BASE_ID, s);
          if (half < 2) tof_base_col_mask[s] |= (half == 0) ? 0x0F : 0xF0;
          tof_baseline_seen[s] = true;
        }
      }
    }
  }
}

void mcp2515BitModify(byte addr, byte mask, byte data) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(TOF_SPI_CS_PIN, LOW);
  SPI.transfer(0x05);
  SPI.transfer(addr);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(TOF_SPI_CS_PIN, HIGH);
  SPI.endTransaction();
}

void serviceTofCanHealth(uint32_t now) {
  if (now - lastTofCanCheck < 2000) return;
  lastTofCanCheck = now;

  byte eflg = TOF_CAN.getError();
  if (eflg & 0xC0) {
    tofRxOverflowTotal++;
    mcp2515BitModify(0x2D, 0xC0, 0x00);
    eflg &= ~0xC0;
  }

  if (eflg & 0x20) {
    if (TOF_CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
      TOF_CAN.setMode(MCP_NORMAL);
    }
  }
}

void pulseNodeReset(uint8_t node) {
  int8_t pin = NODE_RESET_PIN[node];
  if (pin < 0) return;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(2);
  pinMode(pin, INPUT);
}

void updateTofLiveness(uint32_t now) {
  for (int n = 1; n <= 3; n++) {
    bool lost = (now - node_last_seen[n] >= NODE_TIMEOUT_MS);
    if (lost) {
      for (uint8_t k = 0; k < NODE_COUNT[n]; k++) tof_alive[NODE_FIRST[n] + k] = false;
      if (node_online[n]) {
        node_online[n] = false;
        node_reset_attempt[n] = now;
      }
      else if (NODE_RESET_PIN[n] >= 0 && now - node_reset_attempt[n] >= NODE_HW_RESET_MS) {
        node_reset_attempt[n] = now;
        pulseNodeReset(n);
      }
    }
  }
}

void serviceTofBus(uint32_t now) {
  fetchTofCanToRAM();
  processTofRAMQueue();

  if (tof_system_mode > 0 && now - lastTofPing >= 500) {
    lastTofPing = now;
    tofPingCount++;
    byte ping[2] = { tofPingCount, tof_system_mode };
    TOF_CAN.sendMsgBuf(ID_MASTER_PING, 0, 2, ping);
  }

  if (now - lastTofHz >= 1000) {
    for (int n = 1; n <= 3; n++) {
      node_hz[n] = node_cycle_count[n] * 1000.0f / (now - lastTofHz);
      node_cycle_count[n] = 0;
    }
    lastTofHz = now;
  }

  if (now - lastTofDisplay >= 1000) {
    lastTofDisplay = now;
    updateTofLiveness(now);
  }

  publishTof64IfDue(now);
  serviceTofCanHealth(now);
}

// ============================================================
// Wheelchair helper functions
// ============================================================

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

uint8_t signedCommandToByte(int value) {
  value = clampInt(value, -100, 100);
  if (value > -deadband && value < deadband) value = 0;
  return (uint8_t)((int8_t)value);
}

int decodeJoystickByte(uint8_t raw) {
  int value = (int8_t)raw;
  value = clampInt(value, -100, 100);
  if (value > -deadband && value < deadband) value = 0;
  return value;
}

float wrapAngle360(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}

void quaternionToEulerDeg(float qr, float qi, float qj, float qk, float &yawDeg, float &pitchDeg, float &rollDeg) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  yawDeg = atan2(2.0f * (qi * qj + qk * qr), sqi - sqj - sqk + sqr) * RAD_TO_DEG;
  pitchDeg = asin(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
  rollDeg = atan2(2.0f * (qj * qk + qi * qr), -sqi - sqj + sqk + sqr) * RAD_TO_DEG;

  yawDeg = wrapAngle360(yawDeg);
}

void enableImuReport() {
  if (!bno08x.enableReport(imuReportType, imuReportIntervalUs)) {
    safeStatusPrint("STATUS,bno085_enable_report_failed");
  }
  else {
    safeStatusPrint("STATUS,bno085_report_enabled");
  }
}

void updateWheelDirectionsFromXY(int x, int y) {
  int newLeftDirection = 0;
  int newRightDirection = 0;

  if (y > 0) {
    newLeftDirection = +1;
    newRightDirection = +1;
  }
  else if (y < 0) {
    newLeftDirection = -1;
    newRightDirection = -1;
  }
  else {
    if (x > 0) {
      newLeftDirection = +1;
      newRightDirection = -1;
    }
    else if (x < 0) {
      newLeftDirection = -1;
      newRightDirection = +1;
    }
  }

  noInterrupts();
  leftDirection = newLeftDirection;
  rightDirection = newRightDirection;
  interrupts();
}

void updateEncoderDirectionSource() {
  unsigned long nowMs = millis();

  bool serialFresh = lastSerialCommandMs > 0 && (nowMs - lastSerialCommandMs <= SERIAL_SIGN_TIMEOUT_MS);
  bool realJoystickFresh = lastRealJoystickMs > 0 && (nowMs - lastRealJoystickMs <= REAL_JOY_SIGN_TIMEOUT_MS);

  if (serialFresh) updateWheelDirectionsFromXY(serialJoyX, serialJoyY);
  else if (realJoystickFresh) updateWheelDirectionsFromXY(realJoyX, realJoyY);
  else updateWheelDirectionsFromXY(0, 0);
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

  if (wheelchairCanSeen && !waitingStartupDelay && !errorSent) {
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

void readWheelchairCANBus() {
  while (CAN.available()) {
    CanMsg msg = CAN.read();

    if (!wheelchairCanSeen) {
      wheelchairCanSeen = true;
      safeStatusPrint("STATUS,wheelchair_can_traffic_detected");
      safeStatusPrint("STATUS,waiting_for_laptop_serial");

      if (hasReceivedSerialCommand && !waitingStartupDelay && !errorSent) {
        waitingStartupDelay = true;
        startupWaitStartMs = millis();
        safeStatusPrint("STATUS,serial_already_received_startup_wait_5s");
      }
    }

    if (msg.isExtendedId() &&
        (msg.id == READ_JOYSTICK_CAN_ID_1 || msg.id == READ_JOYSTICK_CAN_ID_2) &&
        msg.data_length >= 2) {
      realJoyX = decodeJoystickByte(msg.data[0]);
      realJoyY = decodeJoystickByte(msg.data[1]);
      lastRealJoystickMs = millis();
      updateEncoderDirectionSource();
    }
  }
}

void handleStartSignal() {
  laptopSessionActive = true;
  resetLaptopControlledState();
  initTof64Words();
  tof64_dirty = true;
  sendTofSystemMode(1, false);

  safeStatusPrint("STATUS,start_received");
  safeStatusPrint("STATUS,tof_live_mode_started");
  safeStatusPrint("STATUS,waiting_for_J_x_y");
  safeStatusPrint("FORMAT,DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg");
  safeStatusPrint("FORMAT,TOF64,time_ms,seq,hex64words");
}

void handleStopSignal() {
  resetLaptopControlledState();
  sendTofSystemMode(0, false);
  laptopSessionActive = false;
}

void handleSerialLine(char *line) {
  if (strcmp(line, "START") == 0) {
    handleStartSignal();
    return;
  }

  if (strcmp(line, "STOP") == 0 || strcmp(line, "0") == 0) {
    handleStopSignal();
    return;
  }

  if (!laptopSessionActive) return;

  if (strcmp(line, "LIVE") == 0 || strcmp(line, "1") == 0) {
    sendTofSystemMode(1, false);
    safeStatusPrint("STATUS,tof_live_mode_started");
    return;
  }

  if (strcmp(line, "CAL") == 0 || strcmp(line, "C") == 0 || strcmp(line, "2") == 0) {
    sendTofSystemMode(2, false);
    safeStatusPrint("STATUS,tof_calibration_started");
    return;
  }

  if (strcmp(line, "DRIVE") == 0 || strcmp(line, "D") == 0 || strcmp(line, "3") == 0) {
    sendTofSystemMode(3, false);
    safeStatusPrint("STATUS,tof_drive_mode_started");
    return;
  }

  if (strcmp(line, "B") == 0 || strcmp(line, "BASELINE") == 0) {
    sendTofSystemMode(3, true);
    safeStatusPrint("STATUS,tof_baseline_resend_requested");
    return;
  }

  if (line[0] != 'J') return;

  char *p1 = strchr(line, ',');
  if (p1 == nullptr) return;

  char *p2 = strchr(p1 + 1, ',');
  if (p2 == nullptr) return;

  int x = atoi(p1 + 1);
  int y = atoi(p2 + 1);
  setSerialJoystickCommand(x, y);
}

void readLaptopSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

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
        serialIndex = 0;
      }
    }
  }
}

void handleStartupSequence() {
  if (!laptopSessionActive) return;
  if (!wheelchairCanSeen) return;
  if (!hasReceivedSerialCommand) return;

  if (!waitingStartupDelay && !errorSent) {
    waitingStartupDelay = true;
    startupWaitStartMs = millis();
    safeStatusPrint("STATUS,startup_wait_5s_started");
    return;
  }

  if (waitingStartupDelay && !errorSent) {
    unsigned long nowMs = millis();

    if (nowMs - startupWaitStartMs >= STARTUP_WAIT_MS) {
      CanMsg errorMsg(CanExtendedId(ERROR_CAN_ID), 8, errorData);
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

void sendJoystickCAN() {
  if (!laptopSessionActive || !injectionEnabled) return;

  unsigned long nowMs = millis();
  if (nowMs - lastJoystickSendMs < JOYSTICK_SEND_PERIOD_MS) return;
  lastJoystickSendMs = nowMs;

  updateEncoderDirectionSource();

  if (nowMs - lastSerialCommandMs > SERIAL_TIMEOUT_MS) {
    setNeutralJoystickOutputOnly();
  }

  uint8_t dataToSend[2] = { joyXByte, joyYByte };
  CanMsg joystickMsg(CanExtendedId(SEND_JOYSTICK_CAN_ID), 2, dataToSend);
  CAN.write(joystickMsg);
}

void sendHeartbeatCAN() {
  if (!laptopSessionActive || !injectionEnabled) return;

  unsigned long nowMs = millis();
  if (nowMs - lastHeartbeatSendMs < HEARTBEAT_SEND_PERIOD_MS) return;
  lastHeartbeatSendMs = nowMs;

  CanMsg heartbeatMsg(CanExtendedId(HEARTBEAT_CAN_ID), 7, heartbeatData);
  CAN.write(heartbeatMsg);
}

void sendSerialCAN() {
  if (!laptopSessionActive || !injectionEnabled) return;

  unsigned long nowMs = millis();
  if (nowMs - lastSerialCanSendMs < SERIAL_CAN_SEND_PERIOD_MS) return;
  lastSerialCanSendMs = nowMs;

  CanMsg serialMsg(CanExtendedId(SERIAL_CAN_ID), 8, serialCanData);
  CAN.write(serialMsg);
}

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
    lastYawDeg = wrapAngle360(lastYawDeg);
  }

  yawDeg = lastYawDeg;
  pitchDeg = lastPitchDeg;
  rollDeg = lastRollDeg;
}

void outputData() {
  if (!laptopSessionActive) return;

  unsigned long nowMs = millis();
  if (nowMs - lastSensorOutputMs < SENSOR_OUTPUT_PERIOD_MS) return;
  lastSensorOutputMs = nowMs;

  noInterrupts();
  long leftTicksCopy = leftTicks;
  long rightTicksCopy = rightTicks;
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
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(1000000);

  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 3000) {
    delay(10);
  }

  initTof64Words();

  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);
  delay(10);

  pinMode(leftHallPin, INPUT_PULLUP);
  pinMode(rightHallPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftHallPin), leftHallInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightHallPin), rightHallInterrupt, RISING);

  Wire.begin();
  Wire.setClock(100000);

  if (bno08x.begin_I2C(BNO08X_ADDR, &Wire)) {
    bno_ok = true;
    delay(100);
    enableImuReport();
  }
  else {
    bno_ok = false;
  }

  pinMode(TOF_CAN_INT_PIN, INPUT_PULLUP);
  SPI.begin();

  while (TOF_CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    delay(500);
  }
  TOF_CAN.setMode(MCP_NORMAL);

  if (!CAN.begin(CanBitRate::BR_125k)) {
    while (1) {
      delay(1000);
    }
  }

  setNeutralJoystickOutputOnly();
  updateEncoderDirectionSource();
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  uint32_t now = millis();

  // Keep both CAN buses alive even before START, but do not print or inject.
  readWheelchairCANBus();
  serviceTofBus(now);

  // START/STOP/J/CAL/DRIVE/BASELINE from laptop.
  readLaptopSerial();

  // Wheelchair injection sequence only runs after START and J,x,y.
  handleStartupSequence();
  sendJoystickCAN();
  sendSerialCAN();
  sendHeartbeatCAN();

  // Laptop output only after START.
  outputData();
}
  