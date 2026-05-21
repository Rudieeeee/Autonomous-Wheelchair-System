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

// Direction comes from CAN joystick
// +1 = forward, -1 = backward, 0 = stopped/neutral
volatile int leftDirection = 0;
volatile int rightDirection = 0;

volatile unsigned long lastLeftInterruptTimeUs = 0;
volatile unsigned long lastRightInterruptTimeUs = 0;

// 5 ms debounce
const unsigned long debounceTimeUs = 5000;

// -------------------- CAN joystick --------------------
const uint32_t JOYSTICK_CAN_ID = 0x02000400;

// Last decoded joystick values
int joyX = 0;
int joyY = 0;

// Your joystick encoding:
// 0x64 = +100
// 0x80 = 0
// 0x9C = -100
const uint8_t RAW_ZERO = 0x80;

const int deadband = 5;

// -------------------- Serial output timing --------------------
const unsigned long outputPeriodMs = 50;
unsigned long lastOutputTimeMs = 0;

// -------------------- Decode joystick byte --------------------
int decodeJoystickByte(uint8_t raw)
{
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

// -------------------- Update wheel directions from joystick --------------------
void updateWheelDirectionsFromJoystick()
{
  int newLeftDirection = 0;
  int newRightDirection = 0;

  if (joyY > 0) {
    // Forward: both wheels forward
    newLeftDirection = +1;
    newRightDirection = +1;
  }
  else if (joyY < 0) {
    // Backward: both wheels backward
    newLeftDirection = -1;
    newRightDirection = -1;
  }
  else {
    // Y = 0, rotate around own axis using X
    if (joyX > 0) {
      // Turn right:
      // left wheel forward, right wheel backward
      newLeftDirection = +1;
      newRightDirection = -1;
    }
    else if (joyX < 0) {
      // Turn left:
      // left wheel backward, right wheel forward
      newLeftDirection = -1;
      newRightDirection = +1;
    }
    else {
      // Neutral
      newLeftDirection = 0;
      newRightDirection = 0;
    }
  }

  noInterrupts();
  leftDirection = newLeftDirection;
  rightDirection = newRightDirection;
  interrupts();
}

// -------------------- Read joystick CAN message --------------------
void readJoystickCAN()
{
  if (!CAN.available()) {
    return;
  }

  CanMsg msg = CAN.read();

  if (msg.id != JOYSTICK_CAN_ID) {
    return;
  }

  // Expected CAN data format: XxYy
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

  updateWheelDirectionsFromJoystick();
}

// -------------------- Left Hall interrupt --------------------
void leftHallInterrupt()
{
  unsigned long nowUs = micros();

  if (nowUs - lastLeftInterruptTimeUs > debounceTimeUs) {
    // Add or subtract depending on CAN joystick direction
    leftTicks += leftDirection;
    lastLeftInterruptTimeUs = nowUs;
  }
}

// -------------------- Right Hall interrupt --------------------
void rightHallInterrupt()
{
  unsigned long nowUs = micros();

  if (nowUs - lastRightInterruptTimeUs > debounceTimeUs) {
    // Add or subtract depending on CAN joystick direction
    rightTicks += rightDirection;
    lastRightInterruptTimeUs = nowUs;
  }
}

// -------------------- Output data --------------------
void outputData()
{
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
    Serial.print(rollDeg, 3);
    Serial.print(",");

    Serial.print(joyX);
    Serial.print(",");
    Serial.print(joyY);
    Serial.print(",");

    Serial.print(leftDirectionCopy);
    Serial.print(",");
    Serial.println(rightDirectionCopy);
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("STATUS,starting");

  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);
  delay(10);

  // -------------------- Hall sensors --------------------
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

  // -------------------- I2C / BNO055 --------------------
  // Arduino GIGA default I2C:
  // SDA = D20
  // SCL = D21
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

  // -------------------- CAN --------------------
  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("STATUS,can_init_failed");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,can_begin_success");

  Serial.println("FORMAT,DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg,joyX,joyY,left_dir,right_dir");
}

void loop()
{
  // Continuously update wheel direction from CAN joystick
  readJoystickCAN();

  // Output IMU + real signed encoder ticks
  outputData();
}