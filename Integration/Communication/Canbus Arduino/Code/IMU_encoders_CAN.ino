#include <Arduino.h>
#include <Wire.h>
#include <Arduino_CAN.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int STANDBY_PIN = 7;

// -------------------- BNO055 --------------------
// Address is usually 0x28. Some modules use 0x29.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// -------------------- Hall sensor pins --------------------
const int leftHallPin  = 2;
const int rightHallPin = 3;

// -------------------- Tick counters --------------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile unsigned long lastLeftInterruptTimeUs = 0;
volatile unsigned long lastRightInterruptTimeUs = 0;

// Start with 5 ms debounce.
// Your old 500000 us = 0.5 s, which is too slow for encoder ticks.
const unsigned long debounceTimeUs = 5000;

// -------------------- CAN IDs --------------------
const uint32_t JOYSTICK_CAN_ID = 0x02000400;
const uint32_t STEP_CAN_ID     = 0x0A040000;
const uint32_t LIGHTS_CAN_ID   = 0x0C000303;
const uint32_t PROFILE_ID      = 0x051;

// -------------------- Timing --------------------
const unsigned long outputPeriodMs = 50;   // 20 Hz IMU/encoder output
unsigned long lastOutputTimeMs = 0;

// -------------------- Interrupts --------------------
void leftHallInterrupt() {
  unsigned long nowUs = micros();

  if (nowUs - lastLeftInterruptTimeUs > debounceTimeUs) {
    leftTicks++;
    lastLeftInterruptTimeUs = nowUs;
  }
}

void rightHallInterrupt() {
  unsigned long nowUs = micros();

  if (nowUs - lastRightInterruptTimeUs > debounceTimeUs) {
    rightTicks++;
    lastRightInterruptTimeUs = nowUs;
  }
}

// -------------------- CAN reader --------------------
void readCAN() {
  if (CAN.available()) {
    CanMsg msg = CAN.read();
    uint32_t id = msg.id;

    Serial.print("CAN,");

    if (id == JOYSTICK_CAN_ID) {
      Serial.print("JOYSTICK");
    } 
    else if (id == STEP_CAN_ID) {
      Serial.print("STEP");
    } 
    else if (id == LIGHTS_CAN_ID) {
      Serial.print("LIGHTS");
    } 
    else if (id == PROFILE_ID) {
      Serial.print("PROFILE");
    } 
    else {
      Serial.print("UNKNOWN");
    }

    Serial.print(",ID=0x");
    Serial.print(id, HEX);

    Serial.print(",DLC=");
    Serial.print(msg.data_length);

    Serial.print(",DATA=");

    for (int i = 0; i < msg.data_length; i++) {
      if (msg.data[i] < 0x10) Serial.print("0");
      Serial.print(msg.data[i], HEX);

      if (i < msg.data_length - 1) {
        Serial.print(" ");
      }
    }

    Serial.println();
  }
}

// -------------------- IMU + Encoder output --------------------
void outputImuAndEncoders() {
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

    float yawDeg   = eul.x();  // heading / yaw
    float rollDeg  = eul.y();
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
  // On Arduino GIGA, default Wire pins are:
  // SDA = D20, SCL = D21
  Wire.begin();
  Wire.setClock(100000);   // safer for BNO055

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

  Serial.println("FORMAT,DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg");
  Serial.println("FORMAT,CAN,type,ID,DLC,DATA");
}

void loop() {
  // Read CAN as often as possible
  readCAN();

  // Output IMU + encoder data every 50 ms
  outputImuAndEncoders();
}