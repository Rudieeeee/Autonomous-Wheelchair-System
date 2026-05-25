#include <Arduino.h>
#include <Wire.h>
#include <Arduino_CAN.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int STANDBY_PIN = 7;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

const int leftHallPin = 2;
const int rightHallPin = 3;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile int leftDirection = 0;
volatile int rightDirection = 0;

volatile unsigned long lastLeftInterruptTimeUs = 0;
volatile unsigned long lastRightInterruptTimeUs = 0;

const unsigned long debounceTimeUs = 50000;

const uint32_t JOYSTICK_CAN_ID = 0x02000000;

int joyX = 0;
int joyY = 0;

const int deadband = 5;

bool joystickEmuEnabled = false;
int emuX = 0;
int emuY = 0;

const unsigned long joystickSendPeriodMs = 10;
unsigned long lastJoystickSendTimeMs = 0;

const unsigned long outputPeriodMs = 50;
unsigned long lastOutputTimeMs = 0;

int decodeJoystickByte(uint8_t raw) {
  int value = (int8_t)raw;

  if (value > 100) value = 100;
  if (value < -100) value = -100;

  if (value > -deadband && value < deadband) {
    value = 0;
  }

  return value;
}

uint8_t encodeJoystickByte(int value) {
  if (value > 100) value = 100;
  if (value < -100) value = -100;

  return (uint8_t)((int8_t)value);
}

void updateWheelDirectionsFromJoystick() {
  int newLeftDirection = 0;
  int newRightDirection = 0;

  if (joyY > 0) {
    newLeftDirection = +1;
    newRightDirection = +1;
  }
  else if (joyY < 0) {
    newLeftDirection = -1;
    newRightDirection = -1;
  }
  else {
    if (joyX > 0) {
      newLeftDirection = +1;
      newRightDirection = -1;
    }
    else if (joyX < 0) {
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

  if (msg.data_length < 2) {
    return;
  }

  joyX = decodeJoystickByte(msg.data[0]);
  joyY = decodeJoystickByte(msg.data[1]);

  updateWheelDirectionsFromJoystick();
}

void sendJoystickCAN() {
  if (!joystickEmuEnabled) {
    return;
  }

  unsigned long nowMs = millis();

  if (nowMs - lastJoystickSendTimeMs < joystickSendPeriodMs) {
    return;
  }

  lastJoystickSendTimeMs = nowMs;

  uint8_t data[2];
  data[0] = encodeJoystickByte(emuX);
  data[1] = encodeJoystickByte(emuY);

  CanMsg msg(CanExtendedId(JOYSTICK_CAN_ID), 2, data);

  CAN.write(msg);
}

void readSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command == "x") {
    joystickEmuEnabled = false;
    emuX = 0;
    emuY = 0;
    joyX = 0;
    joyY = 0;
    updateWheelDirectionsFromJoystick();
    Serial.println("STATUS,joystick_emulation_stopped");
    return;
  }

  if (command.startsWith("r")) {
    int speed = command.substring(1).toInt();

    joystickEmuEnabled = true;
    emuX = speed;
    emuY = 0;

    Serial.print("STATUS,rotate_right,");
    Serial.println(speed);
    return;
  }

  if (command.startsWith("l")) {
    int speed = command.substring(1).toInt();

    joystickEmuEnabled = true;
    emuX = -speed;
    emuY = 0;

    Serial.print("STATUS,rotate_left,");
    Serial.println(speed);
    return;
  }

  Serial.println("STATUS,unknown_command");
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

void outputData() {
  unsigned long nowMs = millis();

  if (nowMs - lastOutputTimeMs >= outputPeriodMs) {
    lastOutputTimeMs = nowMs;

    noInterrupts();
    long leftTicksCopy = leftTicks;
    long rightTicksCopy = rightTicks;
    int leftDirectionCopy = leftDirection;
    int rightDirectionCopy = rightDirection;
    interrupts();

    int leftState = digitalRead(leftHallPin);
    int rightState = digitalRead(rightHallPin);

    imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    float yawDeg = eul.x();
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
  Serial.begin(115200);
  while (!Serial);

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

  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("STATUS,can_init_failed");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,can_begin_success");
  Serial.println("FORMAT,DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg");
  Serial.println("COMMANDS,r30,l30,x");
}

void loop() {
  readSerialCommands();
  readJoystickCAN();
  sendJoystickCAN();
  outputData();
}