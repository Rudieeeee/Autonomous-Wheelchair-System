#include <Arduino.h>
#include <Arduino_CAN.h>

const int STANDBY_PIN = 7;

// Joystick CAN ID
const uint32_t JOYSTICK_CAN_ID = 0x82000000;

// Simulated encoder ticks
long leftTicks = 0;
long rightTicks = 0;

// Joystick values
int joyX = 0;
int joyY = 0;

const int deadband = 5;

// Update ticks every 50 ms
const unsigned long tickUpdatePeriodMs = 50;
unsigned long lastTickUpdateMs = 0;

bool joystickReceived = false;

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

  if (msg.data_length < 2) {
    return;
  }

  joyX = decodeJoystickByte(msg.data[0]);
  joyY = decodeJoystickByte(msg.data[1]);

  joystickReceived = true;
}

void updateTicksFromJoystick() {
  unsigned long nowMs = millis();

  if (nowMs - lastTickUpdateMs < tickUpdatePeriodMs) {
    return;
  }

  lastTickUpdateMs = nowMs;

  if (!joystickReceived) {
    return;
  }

  if (joyY > 0) {
    leftTicks++;
    rightTicks++;
  }
  else if (joyY < 0) {
    leftTicks--;
    rightTicks--;
  }
  else {
    if (joyX > 0) {
      leftTicks++;
      rightTicks--;
    }
    else if (joyX < 0) {
      leftTicks--;
      rightTicks++;
    }
  }

  Serial.print(leftTicks);
  Serial.print(",");
  Serial.println(rightTicks);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);
  delay(10);

  if (!CAN.begin(CanBitRate::BR_125k)) {
    while (1) {
      delay(1000);
    }
  }
}

void loop() {
  readJoystickCAN();
  updateTicksFromJoystick();
}