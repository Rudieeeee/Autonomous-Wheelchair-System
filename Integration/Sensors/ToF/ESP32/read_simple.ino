#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_MatrixLidar.h"

#define I2C_SDA 5   // XIAO D4 = GPIO5
#define I2C_SCL 6   // XIAO D5 = GPIO6

DFRobot_MatrixLidar_I2C tof(0x33, &Wire);

uint16_t distances[64];

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Starting DFRobot Matrix ToF on XIAO ESP32-S3...");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (tof.begin() != 0) {
    Serial.println("STATUS,begin_error");
    while (1) delay(1000);
  }

  Serial.println("STATUS,begin_success");

  // Do NOT call setRangingMode for now.
  // Some firmware versions reject this command even though getAllData works.

  Serial.println("FORMAT,8x8_distance_mm");
}

void loop() {
  uint8_t ret = tof.getAllData(distances);

  if (ret != 0) {
    Serial.println("STATUS,getAllData_error");
    delay(500);
    return;
  }

  for (uint8_t row = 0; row < 8; row++) {
    for (uint8_t col = 0; col < 8; col++) {
      uint8_t i = row * 8 + col;

      Serial.print(distances[i]);
      Serial.print("\t");
    }
    Serial.println();
  }

  Serial.println("------------------------------");
  delay(100);
}