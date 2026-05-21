#include <Wire.h>
#include <Adafruit_VL53L7CX.h>

#define I2C_SDA 5   // XIAO D4 = GPIO5
#define I2C_SCL 6   // XIAO D5 = GPIO6

Adafruit_VL53L7CX tof;

// -------------------- Grid settings --------------------
const int GRID_SIZE = 8;
const int ZONES = 64;

// -------------------- Sensor settings --------------------
const int RANGING_FREQ_HZ = 10;

// -------------------- Valid distance limits --------------------
const int MIN_VALID_MM = 50;
const int MAX_VALID_MM = 3000;

// -------------------- Filtering settings --------------------
const int MEDIAN_WINDOW = 5;
int history[ZONES][MEDIAN_WINDOW];
bool historyValid[ZONES][MEDIAN_WINDOW];
int historyIndex = 0;
bool historyFilled = false;

float filtered[ZONES];
bool filteredValid[ZONES];

const float alpha = 0.25;
const int MAX_JUMP_MM = 600;

// -------------------- Output timing --------------------
unsigned long lastOutputMs = 0;
const unsigned long OUTPUT_PERIOD_MS = 100;

// -------------------- Dead output value --------------------
const int NO_OBJECT = -1;

// -------------------- Check valid measurement --------------------
bool isValidMeasurement(VL53L7CX_ResultsData &results, int i) {
  int d = results.distance_mm[i];

  // Status 5 is usually valid/range OK
  if (results.target_status[i] != 5) {
    return false;
  }

  if (d < MIN_VALID_MM || d > MAX_VALID_MM) {
    return false;
  }

  return true;
}

// -------------------- Sort helper for median --------------------
void sortSmallArray(int *arr, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }
}

// -------------------- Get median from zone history --------------------
bool getMedianForZone(int zone, int &medianOut) {
  int values[MEDIAN_WINDOW];
  int count = 0;

  for (int k = 0; k < MEDIAN_WINDOW; k++) {
    if (historyValid[zone][k]) {
      values[count] = history[zone][k];
      count++;
    }
  }

  // Require at least 3 valid values out of 5
  if (count < 3) {
    return false;
  }

  sortSmallArray(values, count);
  medianOut = values[count / 2];
  return true;
}

// -------------------- Update history with new sensor frame --------------------
void updateHistory(VL53L7CX_ResultsData &results) {
  for (int i = 0; i < ZONES; i++) {
    if (isValidMeasurement(results, i)) {
      history[i][historyIndex] = results.distance_mm[i];
      historyValid[i][historyIndex] = true;
    } else {
      history[i][historyIndex] = 0;
      historyValid[i][historyIndex] = false;
    }
  }

  historyIndex++;

  if (historyIndex >= MEDIAN_WINDOW) {
    historyIndex = 0;
    historyFilled = true;
  }
}

// -------------------- Update filtered values --------------------
void updateFilteredValues() {
  for (int i = 0; i < ZONES; i++) {
    int medianValue;

    if (!getMedianForZone(i, medianValue)) {
      continue;
    }

    if (!filteredValid[i]) {
      filtered[i] = medianValue;
      filteredValid[i] = true;
      continue;
    }

    if (abs(medianValue - filtered[i]) > MAX_JUMP_MM) {
      continue;
    }

    filtered[i] = alpha * medianValue + (1.0 - alpha) * filtered[i];
  }
}

// -------------------- Neighbour check --------------------
bool hasValidNeighbour(int zone) {
  int row = zone / GRID_SIZE;
  int col = zone % GRID_SIZE;

  int validNeighbours = 0;

  for (int dr = -1; dr <= 1; dr++) {
    for (int dc = -1; dc <= 1; dc++) {
      if (dr == 0 && dc == 0) continue;

      int nr = row + dr;
      int nc = col + dc;

      if (nr < 0 || nr >= GRID_SIZE || nc < 0 || nc >= GRID_SIZE) {
        continue;
      }

      int neighbourIndex = nr * GRID_SIZE + nc;

      if (filteredValid[neighbourIndex]) {
        int diff = abs((int)filtered[neighbourIndex] - (int)filtered[zone]);

        // Neighbour should be at similar distance
        if (diff < 400) {
          validNeighbours++;
        }
      }
    }
  }

  // Require at least one nearby supporting zone
  return validNeighbours >= 1;
}

// -------------------- Get closest valid object in region --------------------
int getRegionClosest(int startCol, int endCol, int startRow, int endRow, int &confidenceOut) {
  int closest = 9999;
  int validCount = 0;
  int supportedCount = 0;

  for (int row = startRow; row <= endRow; row++) {
    for (int col = startCol; col <= endCol; col++) {
      int i = row * GRID_SIZE + col;

      if (!filteredValid[i]) {
        continue;
      }

      validCount++;

      if (!hasValidNeighbour(i)) {
        continue;
      }

      supportedCount++;

      int d = (int)filtered[i];

      if (d < closest) {
        closest = d;
      }
    }
  }

  confidenceOut = supportedCount;

  if (closest == 9999) {
    return NO_OBJECT;
  }

  return closest;
}

// -------------------- Emergency stop decision --------------------
bool shouldEmergencyStop(int centerDistance, int frontLowDistance) {
  const int CENTER_STOP_MM = 450;
  const int LOW_STOP_MM = 350;

  if (centerDistance != NO_OBJECT && centerDistance < CENTER_STOP_MM) {
    return true;
  }

  if (frontLowDistance != NO_OBJECT && frontLowDistance < LOW_STOP_MM) {
    return true;
  }

  return false;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("STATUS,starting_vl53l7cx_advanced_filter");

  for (int i = 0; i < ZONES; i++) {
    filtered[i] = 0;
    filteredValid[i] = false;

    for (int k = 0; k < MEDIAN_WINDOW; k++) {
      history[i][k] = 0;
      historyValid[i][k] = false;
    }
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
  tof.startRanging();

  Serial.println("FORMAT,time_ms,left_mm,left_conf,center_mm,center_conf,right_mm,right_conf,front_low_mm,front_low_conf,emergency_stop");
}

// -------------------- Loop --------------------
void loop() {
  VL53L7CX_ResultsData results;

  if (tof.isDataReady()) {
    if (tof.getRangingData(&results)) {
      updateHistory(results);
      updateFilteredValues();
    }
  }

  unsigned long nowMs = millis();

  if (nowMs - lastOutputMs >= OUTPUT_PERIOD_MS) {
    lastOutputMs = nowMs;

    int leftConf = 0;
    int centerConf = 0;
    int rightConf = 0;
    int frontLowConf = 0;

    // Region split:
    // left   = columns 0,1,2
    // center = columns 3,4
    // right  = columns 5,6,7
    //
    // front_low = lower half, middle columns
    int leftDistance = getRegionClosest(0, 2, 0, 7, leftConf);
    int centerDistance = getRegionClosest(3, 4, 0, 7, centerConf);
    int rightDistance = getRegionClosest(5, 7, 0, 7, rightConf);
    int frontLowDistance = getRegionClosest(2, 5, 4, 7, frontLowConf);

    bool emergencyStop = shouldEmergencyStop(centerDistance, frontLowDistance);

    Serial.print("TOF,");
    Serial.print(nowMs);
    Serial.print(",");
    Serial.print(leftDistance);
    Serial.print(",");
    Serial.print(leftConf);
    Serial.print(",");
    Serial.print(centerDistance);
    Serial.print(",");
    Serial.print(centerConf);
    Serial.print(",");
    Serial.print(rightDistance);
    Serial.print(",");
    Serial.print(rightConf);
    Serial.print(",");
    Serial.print(frontLowDistance);
    Serial.print(",");
    Serial.print(frontLowConf);
    Serial.print(",");
    Serial.println(emergencyStop ? 1 : 0);
  }

  delay(5);
}