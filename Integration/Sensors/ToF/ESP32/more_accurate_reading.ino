#include <Wire.h>
#include <Adafruit_VL53L7CX.h>

#define I2C_SDA 5   // XIAO D4 = GPIO5
#define I2C_SCL 6   // XIAO D5 = GPIO6

Adafruit_VL53L7CX tof;

// -------------------- Sensor settings --------------------
const int RESOLUTION = 64;        // 8x8
const int GRID_SIZE = 8;
const int RANGING_FREQ_HZ = 10;   // lower = usually more stable

// -------------------- Filtering settings --------------------
float filteredDistance[64];
bool filterInitialized[64];

const float alpha = 0.25;         // lower = smoother, higher = faster response
const int minValidDistance = 50;  // mm
const int maxValidDistance = 3500; // mm, VL53L7CX practical max depends on target
const int maxJumpAllowed = 600;   // mm, reject sudden jumps larger than this

// -------------------- Region output timing --------------------
unsigned long lastOutputMs = 0;
const unsigned long outputPeriodMs = 100; // 10 Hz serial output

bool isValidZone(VL53L7CX_ResultsData &results, int i) {
  int d = results.distance_mm[i];
  int status = results.target_status[i];

  // status 5 is normally valid/range OK
  if (status != 5) return false;

  if (d < minValidDistance) return false;
  if (d > maxValidDistance) return false;

  return true;
}

void updateFilteredDistances(VL53L7CX_ResultsData &results) {
  for (int i = 0; i < RESOLUTION; i++) {
    if (!isValidZone(results, i)) {
      continue;
    }

    float newValue = results.distance_mm[i];

    if (!filterInitialized[i]) {
      filteredDistance[i] = newValue;
      filterInitialized[i] = true;
      continue;
    }

    // Reject sudden large jumps
    if (abs(newValue - filteredDistance[i]) > maxJumpAllowed) {
      continue;
    }

    // Exponential moving average
    filteredDistance[i] =
      alpha * newValue + (1.0 - alpha) * filteredDistance[i];
  }
}

int getRegionClosestDistance(int startCol, int endCol, int startRow, int endRow) {
  int closest = 9999;

  for (int row = startRow; row <= endRow; row++) {
    for (int col = startCol; col <= endCol; col++) {
      int i = row * GRID_SIZE + col;

      if (filterInitialized[i]) {
        int d = (int)filteredDistance[i];

        if (d < closest) {
          closest = d;
        }
      }
    }
  }

  if (closest == 9999) {
    return -1; // no valid object
  }

  return closest;
}

void printFilteredMatrix() {
  Serial.println("FILTERED_MATRIX_MM");

  for (int row = 0; row < GRID_SIZE; row++) {
    for (int col = 0; col < GRID_SIZE; col++) {
      int i = row * GRID_SIZE + col;

      if (filterInitialized[i]) {
        Serial.print((int)filteredDistance[i]);
      } else {
        Serial.print("----");
      }

      Serial.print("\t");
    }

    Serial.println();
  }

  Serial.println("----------------");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("STATUS,starting_vl53l7cx_accurate_mode");

  for (int i = 0; i < RESOLUTION; i++) {
    filteredDistance[i] = 0;
    filterInitialized[i] = false;
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!tof.begin(0x29, &Wire)) {
    Serial.println("STATUS,vl53l7cx_begin_failed");
    Serial.println("STATUS,check_wiring_i2c_address_power");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("STATUS,vl53l7cx_begin_success");

  tof.setResolution(8 * 8);
  tof.setRangingFrequency(RANGING_FREQ_HZ);

  // Optional: strongest target can sometimes be more stable than closest target.
  // For obstacle avoidance, closest target is usually safer.
  // tof.setTargetOrder(VL53L7CX_TARGET_ORDER_CLOSEST);

  tof.startRanging();

  Serial.println("FORMAT,REGION,time_ms,left_mm,center_mm,right_mm,front_low_mm");
}

void loop() {
  VL53L7CX_ResultsData results;

  if (tof.isDataReady()) {
    if (tof.getRangingData(&results)) {
      updateFilteredDistances(results);
    }
  }

  unsigned long nowMs = millis();

  if (nowMs - lastOutputMs >= outputPeriodMs) {
    lastOutputMs = nowMs;

    // Split 8x8 grid into useful regions.
    // Columns:
    // 0 1 2 = left
    // 3 4   = center
    // 5 6 7 = right
    //
    // Rows:
    // 0 top/far part of FOV
    // 7 bottom/near part of FOV
    //
    // You may need to flip rows/columns depending on sensor orientation.

    int leftDistance = getRegionClosestDistance(0, 2, 0, 7);
    int centerDistance = getRegionClosestDistance(3, 4, 0, 7);
    int rightDistance = getRegionClosestDistance(5, 7, 0, 7);

    // Lower half can be useful for footrest / low obstacle detection
    int frontLowDistance = getRegionClosestDistance(2, 5, 4, 7);

    Serial.print("REGION,");
    Serial.print(nowMs);
    Serial.print(",");
    Serial.print(leftDistance);
    Serial.print(",");
    Serial.print(centerDistance);
    Serial.print(",");
    Serial.print(rightDistance);
    Serial.print(",");
    Serial.println(frontLowDistance);

    // Uncomment if you want full filtered matrix.
    // printFilteredMatrix();
  }

  delay(5);
}