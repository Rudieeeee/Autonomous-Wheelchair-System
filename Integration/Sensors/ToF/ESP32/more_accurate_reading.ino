#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_MatrixLidar.h"

#define I2C_SDA 5   // XIAO D4 = GPIO5
#define I2C_SCL 6   // XIAO D5 = GPIO6

DFRobot_MatrixLidar_I2C tof(0x33, &Wire);

const uint8_t GRID_SIZE = 8;
const uint8_t ZONES = 64;

// Change this to 3 or 5
const uint8_t AVG_WINDOW = 5;

uint16_t distances[ZONES];

// History buffer: one rolling window for every zone
uint16_t history[ZONES][AVG_WINDOW];

uint8_t historyIndex = 0;
uint8_t samplesCollected = 0;

uint16_t averagedDistances[ZONES];

void updateRollingAverage() {
  // Store newest frame in history
  for (uint8_t i = 0; i < ZONES; i++) {
    history[i][historyIndex] = distances[i];
  }

  // Move circular buffer index
  historyIndex++;
  if (historyIndex >= AVG_WINDOW) {
    historyIndex = 0;
  }

  // Count how many samples are valid during startup
  if (samplesCollected < AVG_WINDOW) {
    samplesCollected++;
  }

  // Calculate average for every zone
  for (uint8_t i = 0; i < ZONES; i++) {
    uint32_t sum = 0;

    for (uint8_t k = 0; k < samplesCollected; k++) {
      sum += history[i][k];
    }

    averagedDistances[i] = sum / samplesCollected;
  }
}

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
  Serial.println("FORMAT,8x8_rolling_average_distance_mm");
}

void loop() {
  uint8_t ret = tof.getAllData(distances);

  if (ret != 0) {
    Serial.println("STATUS,getAllData_error");
    delay(500);
    return;
  }

  updateRollingAverage();

  for (uint8_t row = 0; row < GRID_SIZE; row++) {
    for (uint8_t col = 0; col < GRID_SIZE; col++) {
      uint8_t i = row * GRID_SIZE + col;

      Serial.print(averagedDistances[i]);
      Serial.print("\t");
    }
    Serial.println();
  }

  Serial.println("------------------------------");
  delay(100);
}