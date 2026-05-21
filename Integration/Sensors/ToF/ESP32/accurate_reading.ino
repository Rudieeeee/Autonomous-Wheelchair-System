#include <Wire.h>
#include <Adafruit_VL53L7CX.h>

#define I2C_SDA 5   // XIAO D4 = GPIO5
#define I2C_SCL 6   // XIAO D5 = GPIO6

Adafruit_VL53L7CX tof;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Starting VL53L7CX on XIAO ESP32-S3...");

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

  tof.setResolution(8 * 8);      // 64 zones
  tof.setRangingFrequency(10);   // stable starting point
  tof.startRanging();

  Serial.println("FORMAT,8x8_distance_mm_valid_only");
}

void loop() {
  VL53L7CX_ResultsData results;

  if (tof.isDataReady()) {
    if (tof.getRangingData(&results)) {

      for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
          int i = row * 8 + col;

          if (results.target_status[i] == 5) {
            Serial.print(results.distance_mm[i]);
          } else {
            Serial.print("----");
          }

          Serial.print("\t");
        }
        Serial.println();
      }

      Serial.println("----------------");
    }
  }

  delay(10);
}