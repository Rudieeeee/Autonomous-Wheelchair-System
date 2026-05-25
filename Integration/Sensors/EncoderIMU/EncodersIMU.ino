#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// BNO055 object.
// Address is usually 0x28. Some modules use 0x29.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// -------------------- Hall sensor pins --------------------
const int leftHallPin = 2;
const int rightHallPin = 3;

// -------------------- Tick counters --------------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Stores the last interrupt time for debounce filtering
volatile unsigned long lastLeftInterruptTimeUs = 0;
volatile unsigned long lastRightInterruptTimeUs = 0;

// Ignore pulses that arrive too close together.
// NOTE: 500000 us = 0.5 seconds. This is very large.
// Use e.g. 2000 to 10000 us if your wheel rotates faster.
const unsigned long debounceTimeUs = 500000;

// -------------------- Serial output timing --------------------
const unsigned long outputPeriodMs = 50;   // 20 Hz output
unsigned long lastOutputTimeMs = 0;

// -------------------- Left Hall interrupt --------------------
void leftHallInterrupt()
{
  unsigned long nowUs = micros();

  if (nowUs - lastLeftInterruptTimeUs > debounceTimeUs) {
    leftTicks++;
    lastLeftInterruptTimeUs = nowUs;
  }
}

// -------------------- Right Hall interrupt --------------------
void rightHallInterrupt()
{
  unsigned long nowUs = micros();

  if (nowUs - lastRightInterruptTimeUs > debounceTimeUs) {
    rightTicks++;
    lastRightInterruptTimeUs = nowUs;
  }
}

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println("STATUS,starting");

  // Hall sensor inputs.
  pinMode(leftHallPin, INPUT_PULLUP);
  pinMode(rightHallPin, INPUT_PULLUP);

  // Start I2C.
  // For Arduino GIGA default Wire pins are D20 SDA and D21 SCL.
  // For ESP32-S3 use Wire.begin(SDA_PIN, SCL_PIN);
  Wire.begin();

  // Optional but useful
  Wire.setClock(400000);

  // Start BNO055
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("STATUS,bno_begin_failed");
    Serial.println("STATUS,check_wiring_or_i2c_address");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,bno_begin_success");

  // BNO055 needs a short delay after mode setup
  delay(1000);

  // Use external crystal if available on your module.
  // Most Adafruit boards have this. Many other breakout boards also do.
  bno.setExtCrystalUse(true);

  // Attach interrupts for wheel tick counting.
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

  Serial.println(
    "FORMAT,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg"
  );
}

void loop()
{
  unsigned long nowMs = millis();

  if (nowMs - lastOutputTimeMs >= outputPeriodMs) {
    lastOutputTimeMs = nowMs;

    // Copy tick counters safely because they are changed inside interrupts.
    noInterrupts();
    long leftTicksCopy = leftTicks;
    long rightTicksCopy = rightTicks;
    interrupts();

    // Raw Hall sensor states.
    int leftState = digitalRead(leftHallPin);
    int rightState = digitalRead(rightHallPin);

    // Read IMU Euler angles.
    // Adafruit returns:
    // x = heading/yaw
    // y = roll
    // z = pitch
    imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    binaryFloat yawDeg, pitchDeg, rollDeg;
    yawDeg.floatingPoint = eul.x();
    rollDeg.floatingPoint = eul.y();
    pitchDeg.floatingPoint = eul.z();

    // CSV output for ROS2 serial reader
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
    Serial.print(yawDeg.floatingPoint, 3);
    Serial.print(",");
    Serial.print(pitchDeg.floatingPoint, 3);
    Serial.print(",");
    Serial.println(rollDeg.floatingPoint, 3);
  }
}