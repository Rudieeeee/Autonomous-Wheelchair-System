#include <Arduino.h>
#include <Wire.h>
#include <Arduino_CAN.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// -------------------- BNO055 --------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// -------------------- Serial output timing --------------------
// 20 ms = 50 Hz. This is good for EKF and SLAM.
const unsigned long outputPeriodUs = 2500;
unsigned long lastOutputTimeUs = 0;


void outputData() {
  unsigned long nowUs = micros();

  if (nowUs - lastOutputTimeUs < outputPeriodUs) {
    return;
  }

  lastOutputTimeUs = nowUs;

 

  // BNO055 fused orientation
  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // BNO055 gyro, normally rad/s in Adafruit library
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // BNO055 acceleration, m/s^2
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float yawDeg = eul.x();
  float rollDeg = eul.y();
  float pitchDeg = eul.z();

  float gyroX = gyro.x();
  float gyroY = gyro.y();
  float gyroZ = gyro.z();

  float accelX = accel.x();
  float accelY = accel.y();
  float accelZ = accel.z();

  uint8_t calSys = 0;
  uint8_t calGyro = 0;
  uint8_t calAccel = 0;
  uint8_t calMag = 0;

  bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

  Serial.print("DATA,");
  Serial.print(nowUs);
  Serial.print(",");

  Serial.print(gyroX, 6);
  Serial.print(",");
  Serial.print(gyroY, 6);
  Serial.print(",");
  Serial.print(gyroZ, 6);
  Serial.print(",");

  Serial.print(accelX, 6);
  Serial.print(",");
  Serial.print(accelY, 6);
  Serial.print(",");
  Serial.print(accelZ, 6);
  Serial.print(",");

  Serial.print(yawDeg, 3);
  Serial.print(",");
  Serial.print(pitchDeg, 3);
  Serial.print(",");
  Serial.print(rollDeg, 3);
  Serial.print(",");

  Serial.print(calSys);
  Serial.print(",");
  Serial.print(calGyro);
  Serial.print(",");
  Serial.print(calAccel);
  Serial.print(",");
  Serial.println(calMag);
}


void setup() {
  Serial.begin(460800);

  // Do not block forever if Serial Monitor is not open
  unsigned long startWaitMs = millis();
  while (!Serial && millis() - startWaitMs < 3000) {
    delay(10);
  }

  Serial.println("STATUS,starting");

  // BNO055 on Arduino GIGA default I2C:
  // SDA = D20, SCL = D21
  Wire.begin();
  Wire.setClock(100000);

  // For wheelchair mapping, IMUPLUS is often better than NDOF
  // because it does not rely on the magnetometer for yaw.
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("STATUS,bno_begin_failed");
    Serial.println("STATUS,check_wiring_or_i2c_address");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,bno_begin_success");
  Serial.println("STATUS,bno_mode_IMUPLUS");

  delay(1000);
  bno.setExtCrystalUse(true);

  
  Serial.println(
    "FORMAT,DATA,time_ms,"
    "gyro_x_radps,gyro_y_radps,gyro_z_radps,"
    "accel_x,accel_y,accel_z,"
    "yaw_deg,pitch_deg,roll_deg,"
    "cal_sys,cal_gyro,cal_accel,cal_mag"
  );
}


void loop() {
  outputData();
}