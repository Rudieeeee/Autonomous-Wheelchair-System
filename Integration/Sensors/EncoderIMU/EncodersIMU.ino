#include <Wire.h>
#include "DFRobot_BNO055.h"

// Short name for the DFRobot BNO055 I2C class
typedef DFRobot_BNO055_IIC BNO;

// BNO055 object on the default I2C bus.
// On Arduino GIGA, default I2C is SDA = D20, SCL = D21.
BNO bno(&Wire, 0x28);

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
// This reduces false double-counting.
const unsigned long debounceTimeUs = 3000;

// -------------------- Serial output timing --------------------
const unsigned long outputPeriodMs = 50;   // 20 Hz output
unsigned long lastOutputTimeMs = 0;

// -------------------- BNO055 status helper --------------------
void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch (eStatus) {
    case BNO::eStatusOK:
      Serial.println("STATUS,everything_ok");
      break;

    case BNO::eStatusErr:
      Serial.println("STATUS,unknown_error");
      break;

    case BNO::eStatusErrDeviceNotDetect:
      Serial.println("STATUS,device_not_detected");
      break;

    case BNO::eStatusErrDeviceReadyTimeOut:
      Serial.println("STATUS,device_ready_timeout");
      break;

    case BNO::eStatusErrDeviceStatus:
      Serial.println("STATUS,device_internal_status_error");
      break;

    default:
      Serial.println("STATUS,unknown_status");
      break;
  }
}

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

void setup()
{
  Serial.begin(115200);

  // Wait briefly for USB serial to become available.
  // This is useful on boards like the GIGA.
  delay(1500);

  Serial.println("STATUS,starting");

  // Hall sensor inputs.
  // INPUT_PULLUP means the pin is normally HIGH.
  // When the Hall sensor pulls the signal LOW, a falling edge occurs.
  pinMode(leftHallPin, INPUT_PULLUP);
  pinMode(rightHallPin, INPUT_PULLUP);

  // Start I2C.
  // On GIGA default Wire pins are D20 SDA and D21 SCL.
  Wire.begin();

  // Start BNO055
  bno.reset();

  while (bno.begin() != BNO::eStatusOK) {
    Serial.println("STATUS,bno_begin_failed");
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }

  Serial.println("STATUS,bno_begin_success");

  // NDOF mode gives fused orientation from accel + gyro + magnetometer.
  bno.setOprMode(BNO::eOprModeNdof);

  // Attach interrupts for wheel tick counting.
  attachInterrupt(
    digitalPinToInterrupt(leftHallPin),
    leftHallInterrupt,
    FALLING
  );

  attachInterrupt(
    digitalPinToInterrupt(rightHallPin),
    rightHallInterrupt,
    FALLING
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
    // Usually: 1 = no magnet, 0 = magnet detected.
    int leftState = digitalRead(leftHallPin);
    int rightState = digitalRead(rightHallPin);

    // Read IMU Euler angles.
    BNO::sEulAnalog_t eul = bno.getEul();

    float yawDeg = eul.head;
    float pitchDeg = eul.pitch;
    float rollDeg = eul.roll;

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
    Serial.print(yawDeg, 3);
    Serial.print(",");
    Serial.print(pitchDeg, 3);
    Serial.print(",");
    Serial.println(rollDeg, 3);
  }
}