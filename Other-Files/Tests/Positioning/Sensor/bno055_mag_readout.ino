#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Initialize with default I2C address 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!bno.begin()) {
    Serial.println("Error: BNO055 not detected check wiring!");
    while (1);
  }
  
  // Use manual fusion-off mode to get raw data for analysis
  // Alternative: use OPERATION_MODE_NDOF if you want internally fused data
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG); 
  delay(1000);
}

void loop() {
  // Read sensor data events
  sensors_event_t accData, gyroData, magData;
  bno.getEvent(&accData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Print unified CSV line over serial
  Serial.print(millis()); Serial.print(",");
  Serial.print(accData.acceleration.x); Serial.print(",");
  Serial.print(accData.acceleration.y); Serial.print(",");
  Serial.print(accData.acceleration.z); Serial.print(",");
  Serial.print(gyroData.gyro.x); Serial.print(",");
  Serial.print(gyroData.gyro.y); Serial.print(",");
  Serial.print(gyroData.gyro.z); Serial.print(",");
  Serial.print(magData.magnetic.x); Serial.print(",");
  Serial.print(magData.magnetic.y); Serial.print(",");
  Serial.println(magData.magnetic.z);

  delay(10); // Aim for ~100Hz output rate
}