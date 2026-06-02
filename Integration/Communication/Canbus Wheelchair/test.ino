#include <Arduino.h>
#include <Arduino_CAN.h>

// ============================================================
// CAN IDs
// ============================================================

const uint32_t ERROR_CAN_ID            = 0x0C000100;
const uint32_t JOYSTICK_CAN_ID         = 0x82000000;

const uint32_t STEP_CAN_ID             = 0x0A040000;
const uint32_t DEVICE_HEARTBEAT_CAN_ID = 0x03C30F0F;

// Serial heartbeat:
// 00E#15C00DE700000000
const uint32_t SERIAL_HEARTBEAT_CAN_ID = 0x00E;

// ============================================================
// State
// ============================================================

bool canSeen = false;
bool errorSent = false;

// ============================================================
// Timing
// ============================================================

unsigned long lastJoystickSendUs = 0;

unsigned long lastStepSendMs = 0;
unsigned long lastDeviceHeartbeatSendMs = 0;
unsigned long lastSerialHeartbeatSendMs = 0;

const unsigned long JOYSTICK_PERIOD_US = 10000;  // 10 ms
const unsigned long STEP_PERIOD_MS = 684;        // 684 ms
const unsigned long DEVICE_HEARTBEAT_PERIOD_MS = 100;
const unsigned long SERIAL_HEARTBEAT_PERIOD_MS = 50;

// Print limiter so Serial does not become too spammy
unsigned long lastReadPrintMs = 0;
unsigned long lastJoystickPrintMs = 0;
unsigned long lastStepPrintMs = 0;
unsigned long lastDeviceHeartbeatPrintMs = 0;
unsigned long lastSerialHeartbeatPrintMs = 0;

const unsigned long READ_PRINT_PERIOD_MS = 100;
const unsigned long JOYSTICK_PRINT_PERIOD_MS = 100;
const unsigned long STEP_PRINT_PERIOD_MS = 1000;
const unsigned long DEVICE_HEARTBEAT_PRINT_PERIOD_MS = 1000;
const unsigned long SERIAL_HEARTBEAT_PRINT_PERIOD_MS = 1000;

// ============================================================
// Data
// ============================================================

uint8_t joystickX = 0x64;
uint8_t joystickY = 0x00;

uint8_t speedByte = 0x00;

// 03C30F0F#87878787878787
uint8_t deviceHeartbeatData[7] = {
  0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87
};

// 00E#15C00DE700000000
uint8_t serialHeartbeatData[8] = {
  0x15, 0xC0, 0x0D, 0xE7,
  0x00, 0x00, 0x00, 0x00
};

// ============================================================
// Print helpers
// ============================================================

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print("0");
  }
  Serial.print(value, HEX);
}

void printCanId(uint32_t id, bool extended) {
  if (extended) {
    Serial.print("EXT 0x");

    if (id < 0x10000000) Serial.print("0");
    if (id < 0x01000000) Serial.print("0");
    if (id < 0x00100000) Serial.print("0");
    if (id < 0x00010000) Serial.print("0");
    if (id < 0x00001000) Serial.print("0");
    if (id < 0x00000100) Serial.print("0");
    if (id < 0x00000010) Serial.print("0");

    Serial.print(id, HEX);
  } else {
    Serial.print("STD 0x");

    if (id < 0x100) Serial.print("0");
    if (id < 0x010) Serial.print("0");

    Serial.print(id, HEX);
  }
}

bool idStartsWith82(uint32_t id) {
  return ((id & 0xFF000000) == 0x82000000);
}

void printCanFrame(
  const char *prefix,
  uint32_t id,
  bool extended,
  uint8_t len,
  const uint8_t *data
) {
  Serial.print(prefix);
  Serial.print(",");

  printCanId(id, extended);

  Serial.print(",LEN=");
  Serial.print(len);
  Serial.print(",DATA=");

  for (uint8_t i = 0; i < len; i++) {
    printHexByte(data[i]);

    if (i + 1 < len) {
      Serial.print(" ");
    }
  }

  Serial.println();
}

// ============================================================
// Send frames
// ============================================================

void sendErrorFrame() {
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

  printCanFrame(
    "SENT_ERROR",
    ERROR_CAN_ID,
    true,
    8,
    errorData
  );
}

void sendJoystickFrame() {
  uint8_t joystickData[2] = {
    joystickX,
    joystickY
  };

  CanMsg joystickMsg(
    CanExtendedId(JOYSTICK_CAN_ID),
    2,
    joystickData
  );

  CAN.write(joystickMsg);

  unsigned long nowMs = millis();

  if (nowMs - lastJoystickPrintMs >= JOYSTICK_PRINT_PERIOD_MS) {
    lastJoystickPrintMs = nowMs;

    printCanFrame(
      "SENT_JOYSTICK",
      JOYSTICK_CAN_ID,
      true,
      2,
      joystickData
    );
  }
}

void sendStepFrame() {
  uint8_t stepData[1] = {
    speedByte
  };

  CanMsg stepMsg(
    CanExtendedId(STEP_CAN_ID),
    1,
    stepData
  );

  CAN.write(stepMsg);

  unsigned long nowMs = millis();

  if (nowMs - lastStepPrintMs >= STEP_PRINT_PERIOD_MS) {
    lastStepPrintMs = nowMs;

    printCanFrame(
      "SENT_STEP",
      STEP_CAN_ID,
      true,
      1,
      stepData
    );
  }
}

void sendDeviceHeartbeatFrame() {
  CanMsg heartbeatMsg(
    CanExtendedId(DEVICE_HEARTBEAT_CAN_ID),
    7,
    deviceHeartbeatData
  );

  CAN.write(heartbeatMsg);

  unsigned long nowMs = millis();

  if (nowMs - lastDeviceHeartbeatPrintMs >= DEVICE_HEARTBEAT_PRINT_PERIOD_MS) {
    lastDeviceHeartbeatPrintMs = nowMs;

    printCanFrame(
      "SENT_DEVICE_HEARTBEAT",
      DEVICE_HEARTBEAT_CAN_ID,
      true,
      7,
      deviceHeartbeatData
    );
  }
}

void sendSerialHeartbeatFrame() {
  CanMsg serialHeartbeatMsg(
    CanStandardId(SERIAL_HEARTBEAT_CAN_ID),
    8,
    serialHeartbeatData
  );

  CAN.write(serialHeartbeatMsg);

  unsigned long nowMs = millis();

  if (nowMs - lastSerialHeartbeatPrintMs >= SERIAL_HEARTBEAT_PRINT_PERIOD_MS) {
    lastSerialHeartbeatPrintMs = nowMs;

    printCanFrame(
      "SENT_SERIAL_HEARTBEAT",
      SERIAL_HEARTBEAT_CAN_ID,
      false,
      8,
      serialHeartbeatData
    );
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  unsigned long serialStart = millis();

  while (!Serial && millis() - serialStart < 3000) {
    delay(10);
  }

  Serial.println("STATUS,starting");

  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("STATUS,can_begin_failed");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,can_begin_success");
  Serial.println("STATUS,waiting_for_any_can_data");
}

// ============================================================
// Loop
// ============================================================

void loop() {
  while (CAN.available()) {
    CanMsg msg = CAN.read();
    canSeen = true;

    uint32_t readId = msg.id;
    bool extended = msg.isExtendedId();

    // Print only IDs that start with 82, for example 0x82000000.
    if (idStartsWith82(readId)) {
      unsigned long nowMs = millis();

      if (nowMs - lastReadPrintMs >= READ_PRINT_PERIOD_MS) {
        lastReadPrintMs = nowMs;

        printCanFrame(
          "READ_ID_STARTS_82",
          readId,
          extended,
          msg.data_length,
          msg.data
        );
      }
    }
  }

  // After any CAN data is seen:
  // 1. Send error once
  // 2. Start joystick + step + heartbeat loops
  if (canSeen && !errorSent) {
    sendErrorFrame();
    errorSent = true;

    lastJoystickSendUs = micros();
    lastStepSendMs = millis();
    lastDeviceHeartbeatSendMs = millis();
    lastSerialHeartbeatSendMs = millis();

    Serial.println("STATUS,error_sent_joystick_step_heartbeats_started");
  }

  if (errorSent) {
    unsigned long nowUs = micros();
    unsigned long nowMs = millis();

    // Joystick every 10 ms
    if (nowUs - lastJoystickSendUs >= JOYSTICK_PERIOD_US) {
      lastJoystickSendUs = nowUs;
      sendJoystickFrame();
    }

    // Speed / step frame every 684 ms
    if (nowMs - lastStepSendMs >= STEP_PERIOD_MS) {
      lastStepSendMs = nowMs;
      sendStepFrame();
    }

    // Device heartbeat every 100 ms
    if (nowMs - lastDeviceHeartbeatSendMs >= DEVICE_HEARTBEAT_PERIOD_MS) {
      lastDeviceHeartbeatSendMs = nowMs;
      sendDeviceHeartbeatFrame();
    }

    // Serial heartbeat every 50 ms
    if (nowMs - lastSerialHeartbeatSendMs >= SERIAL_HEARTBEAT_PERIOD_MS) {
      lastSerialHeartbeatSendMs = nowMs;
      sendSerialHeartbeatFrame();
    }
  }
}