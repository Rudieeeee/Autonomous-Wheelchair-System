#include <Arduino.h>
#include <Arduino_CAN.h>
const int STANDBY_PIN = 7;

// -------------------- CAN ID --------------------
const uint32_t JOYSTICK_CAN_ID = 0x02000400;

// -------------------- Simulated encoder ticks --------------------
long leftTicks = 0;
long rightTicks = 0;

// -------------------- Joystick values --------------------
int joyX = 0;
int joyY = 0;

// Your joystick encoding:
// 0x64 = +100
// 0x80 = 0
// 0x9C = -100
const uint8_t RAW_POS_100 = 0x64;
const uint8_t RAW_ZERO    = 0x80;
const uint8_t RAW_NEG_100 = 0x9C;

// Ignore small joystick noise around zero
const int deadband = 5;

// Update ticks every 50 ms
const unsigned long tickUpdatePeriodMs = 50;
unsigned long lastTickUpdateMs = 0;

// Set true after first joystick message is received
bool joystickReceived = false;

// -------------------- Decode joystick byte --------------------
int decodeJoystickByte(uint8_t raw) {
  // raw 0x64 = +100
  // raw 0x80 = 0
  // raw 0x9C = -100
  int value = (RAW_ZERO - (int)raw) * 100 / 28;

  if (value > 100) value = 100;
  if (value < -100) value = -100;

  if (value > -deadband && value < deadband) {
    value = 0;
  }

  return value;
}

// -------------------- Read joystick CAN message --------------------
void readJoystickCAN() {
  if (!CAN.available()) {
    return;
  }

  CanMsg msg = CAN.read();

  // Only use joystick CAN message
  if (msg.id != JOYSTICK_CAN_ID) {
    return;
  }

  // Expected data format: XxYy
  // data[0] = X label / unused
  // data[1] = X value
  // data[2] = Y label / unused
  // data[3] = Y value
  if (msg.data_length < 4) {
    return;
  }

  uint8_t rawX = msg.data[1];
  uint8_t rawY = msg.data[3];

  joyX = decodeJoystickByte(rawX);
  joyY = decodeJoystickByte(rawY);

  joystickReceived = true;

  Serial.print("JOYSTICK,rawX=0x");
  if (rawX < 0x10) Serial.print("0");
  Serial.print(rawX, HEX);

  Serial.print(",rawY=0x");
  if (rawY < 0x10) Serial.print("0");
  Serial.print(rawY, HEX);

  Serial.print(",x=");
  Serial.print(joyX);

  Serial.print(",y=");
  Serial.println(joyY);
}

// -------------------- Update ticks from joystick --------------------
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
    // Forward
    leftTicks++;
    rightTicks++;
  }
  else if (joyY < 0) {
    // Backward
    leftTicks--;
    rightTicks--;
  }
  else {
    // Y = 0, rotate around own axis based on X
    if (joyX > 0) {
      // Turn right:
      // left wheel forward, right wheel backward
      leftTicks++;
      rightTicks--;
    }
    else if (joyX < 0) {
      // Turn left:
      // left wheel backward, right wheel forward
      leftTicks--;
      rightTicks++;
    }
  }

  Serial.print("TICKS,left=");
  Serial.print(leftTicks);
  Serial.print(",right=");
  Serial.println(rightTicks);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("STATUS,starting_joystick_can_tick_reader");

  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);
  delay(10);

  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("STATUS,can_init_failed");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,can_begin_success");
  Serial.println("FORMAT,JOYSTICK,rawX,rawY,x,y");
  Serial.println("FORMAT,TICKS,left,right");
}

void loop() {
  readJoystickCAN();
  updateTicksFromJoystick();
}