#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_MatrixLidar.h"

#define I2C_SDA 5   // XIAO D4 = GPIO5
#define I2C_SCL 6   // XIAO D5 = GPIO6

DFRobot_MatrixLidar_I2C tof(0x33, &Wire);

// -------------------- Matrix settings --------------------
const int GRID_SIZE = 8;
const int ZONES = 64;

// -------------------- Raw sensor data --------------------
uint16_t distances[ZONES];

// -------------------- Valid distance limits --------------------
const int MIN_VALID_MM = 50;
const int MAX_VALID_MM = 3500;

// -------------------- Rolling average settings --------------------
const int WINDOW_SIZE = 5;

uint16_t history[ZONES][WINDOW_SIZE];
bool historyValid[ZONES][WINDOW_SIZE];
int historyIndex = 0;

// -------------------- Filtered output --------------------
float filtered[ZONES];
bool filteredValid[ZONES];

// Exponential moving average.
// Smaller = smoother but slower.
// Bigger = faster but more noisy.
const float ALPHA = 0.25;

// Reject sudden changes larger than this compared to previous filtered value.
const int MAX_JUMP_MM = 600;

// -------------------- Timing --------------------
unsigned long lastPrintMs = 0;
const unsigned long PRINT_PERIOD_MS = 100;

// -------------------- Check valid raw distance --------------------
bool isValidDistance(uint16_t d) {
  if (d < MIN_VALID_MM) return false;
  if (d > MAX_VALID_MM) return false;
  return true;
}

// -------------------- Add current frame to history --------------------
void updateHistory() {
  for (int i = 0; i < ZONES; i++) {
    if (isValidDistance(distances[i])) {
      history[i][historyIndex] = distances[i];
      historyValid[i][historyIndex] = true;
    } else {
      history[i][historyIndex] = 0;
      historyValid[i][historyIndex] = false;
    }
  }

  historyIndex++;
  if (historyIndex >= WINDOW_SIZE) {
    historyIndex = 0;
  }
}

// -------------------- Rolling average for one zone --------------------
bool getRollingAverage(int zone, float &avgOut) {
  long sum = 0;
  int count = 0;

  for (int k = 0; k < WINDOW_SIZE; k++) {
    if (historyValid[zone][k]) {
      sum += history[zone][k];
      count++;
    }
  }

  // Require at least 3 valid samples out of 5
  if (count < 3) {
    return false;
  }

  avgOut = (float)sum / count;
  return true;
}

// -------------------- Neighbour support check --------------------
bool hasNeighbourSupport(int zone) {
  int row = zone / GRID_SIZE;
  int col = zone % GRID_SIZE;

  if (!filteredValid[zone]) {
    return false;
  }

  int supportedNeighbours = 0;

  for (int dr = -1; dr <= 1; dr++) {
    for (int dc = -1; dc <= 1; dc++) {
      if (dr == 0 && dc == 0) continue;

      int nr = row + dr;
      int nc = col + dc;

      if (nr < 0 || nr >= GRID_SIZE || nc < 0 || nc >= GRID_SIZE) {
        continue;
      }

      int ni = nr * GRID_SIZE + nc;

      if (!filteredValid[ni]) {
        continue;
      }

      if (abs((int)filtered[ni] - (int)filtered[zone]) < 400) {
        supportedNeighbours++;
      }
    }
  }

  // At least one nearby zone should agree
  return supportedNeighbours >= 1;
}

// -------------------- Update filtered values --------------------
void updateFilteredValues() {
  for (int i = 0; i < ZONES; i++) {
    float avgValue;

    if (!getRollingAverage(i, avgValue)) {
      continue;
    }

    if (!filteredValid[i]) {
      filtered[i] = avgValue;
      filteredValid[i] = true;
      continue;
    }

    // Spike / jump rejection
    if (abs((int)avgValue - (int)filtered[i]) > MAX_JUMP_MM) {
      continue;
    }

    // Exponential smoothing after rolling average
    filtered[i] = ALPHA * avgValue + (1.0 - ALPHA) * filtered[i];
  }
}

// -------------------- Print filtered matrix --------------------
void printFilteredMatrix() {
  for (uint8_t row = 0; row < GRID_SIZE; row++) {
    for (uint8_t col = 0; col < GRID_SIZE; col++) {
      uint8_t i = row * GRID_SIZE + col;

      if (filteredValid[i] && hasNeighbourSupport(i)) {
        Serial.print((int)filtered[i]);
      } else if (filteredValid[i]) {
        // If you want to keep isolated detections, replace this with:
        // Serial.print((int)filtered[i]);
        Serial.print("----");
      } else {
        Serial.print("----");
      }

      Serial.print("\t");
    }

    Serial.println();
  }

  Serial.println("------------------------------");
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Starting DFRobot Matrix ToF on XIAO ESP32-S3 with filtering...");

  for (int i = 0; i < ZONES; i++) {
    filtered[i] = 0;
    filteredValid[i] = false;

    for (int k = 0; k < WINDOW_SIZE; k++) {
      history[i][k] = 0;
      historyValid[i][k] = false;
    }
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (tof.begin() != 0) {
    Serial.println("STATUS,begin_error");
    while (1) delay(1000);
  }

  Serial.println("STATUS,begin_success");
  Serial.println("FORMAT,filtered_8x8_distance_mm");
}

// -------------------- Loop --------------------
void loop() {
  uint8_t ret = tof.getAllData(distances);

  if (ret != 0) {
    Serial.println("STATUS,getAllData_error");
    delay(500);
    return;
  }

  updateHistory();
  updateFilteredValues();

  unsigned long nowMs = millis();

  if (nowMs - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = nowMs;
    printFilteredMatrix();
  }

  delay(10);
}