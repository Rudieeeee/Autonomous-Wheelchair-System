#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "driver/twai.h"
#include "DFRobot_MatrixLidar.h"

#define CAN_TX_PIN GPIO_NUM_43
#define CAN_RX_PIN GPIO_NUM_44

#define ID_SYSTEM_CTRL   0x010
#define ID_TOF1_BASE     0x110  
#define ID_TOF2_BASE     0x120  
#define ID_TOF3_BASE     0x130  
#define ID_MASTER_PING   0x400
#define NODE_ID          0x403

#define I2C_SDA 5
#define I2C_SCL 6

DFRobot_MatrixLidar_I2C tof1(0x31, &Wire);
DFRobot_MatrixLidar_I2C tof2(0x32, &Wire);
DFRobot_MatrixLidar_I2C tof3(0x33, &Wire);

const float vertical_angles[8] = { 26.25, 18.75, 11.25, 3.75, -3.75, -11.25, -18.75, -26.25 };
const float ALPHA = 0.9;

uint16_t distances1[64], distances2[64], distances3[64];
float filtered1[64], filtered2[64], filtered3[64];
bool init1 = false, init2 = false, init3 = false;

bool system_active = false;
unsigned long lastToFUpdate = 0;
unsigned long lastDisplayTime = 0;

uint16_t local_matrix[3][8]; 

void setup() {
  Serial.begin(115200);
  delay(1500);
  
  for (int s = 0; s < 3; s++) {
    for (int c = 0; c < 8; c++) {
      local_matrix[s][c] = 4000;
    }
  }

  Serial.println("ESP32 Node 3 Ready. Rate-Locked for Millimeter Sync.");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  if (tof1.begin() != 0) Serial.println("!! TOF1_BEGIN_ERROR !!");
  if (tof2.begin() != 0) Serial.println("!! TOF2_BEGIN_ERROR !!");
  if (tof3.begin() != 0) Serial.println("!! TOF3_BEGIN_ERROR !!");
}

void processAndTransmitMatrix(uint8_t sensorIdx, DFRobot_MatrixLidar_I2C& tof, uint16_t distances[64], float filtered[64], bool& initialized, uint16_t can_base_id) {
  if (tof.getAllData(distances) != 0) return;

  for (uint8_t row = 0; row < 8; row++) {
    float cos_angle = cos(vertical_angles[row] * PI / 180.0);
    for (uint8_t col = 0; col < 8; col++) {
      uint8_t i = row * 8 + col;
      float projected = distances[i] * cos_angle;
      if (distances[i] == 0) projected = 4000.0; 

      if (!initialized) filtered[i] = projected;
      filtered[i] = ALPHA * projected + (1.0f - ALPHA) * filtered[i];
    }
  }
  initialized = true;

  uint16_t column_packets[8];
  for (uint8_t col = 0; col < 8; col++) {
    uint16_t min_dist = 65535;
    bool valid_found = false;

    for (uint8_t row = 0; row < 8; row++) {
      uint16_t value = (uint16_t)filtered[row * 8 + col];
      if (value > 20 && value < 3500 && value < min_dist) {
        min_dist = value;
        valid_found = true;
      }
    }

    if (!valid_found) min_dist = 4000;
    local_matrix[sensorIdx][col] = min_dist;
    column_packets[col] = (min_dist << 3) | (col & 0x07);
  }

  // Frame A
  twai_message_t tx_msg;
  tx_msg.extd = 0; tx_msg.rtr = 0;
  tx_msg.data_length_code = 8;
  tx_msg.identifier = can_base_id;
  for (int i = 0; i < 4; i++) {
    tx_msg.data[i * 2]     = column_packets[i] & 0xFF;
    tx_msg.data[i * 2 + 1] = (column_packets[i] >> 8) & 0xFF;
  }
  twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
  delayMicroseconds(200);

  // Frame B
  tx_msg.identifier = can_base_id + 1;
  for (int i = 0; i < 4; i++) {
    tx_msg.data[i * 2]     = column_packets[i + 4] & 0xFF;
    tx_msg.data[i * 2 + 1] = (column_packets[i + 4] >> 8) & 0xFF;
  }
  twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
}

void loop() {
  twai_message_t rx_msg;
  
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(0)) == ESP_OK) {
    if (rx_msg.identifier == ID_SYSTEM_CTRL && rx_msg.data_length_code > 0) {
      system_active = (rx_msg.data[0] == 1);
      Serial.println(system_active ? ">>> SYSTEM AWOKE <<<" : ">>> SYSTEM ASLEEP <<<");
    }
    else if (system_active && rx_msg.identifier == ID_MASTER_PING) {
      twai_message_t tx_msg = rx_msg;
      tx_msg.identifier = NODE_ID;
      twai_transmit(&tx_msg, pdMS_TO_TICKS(50));
    }
  }

  // CRITICAL CHANGE: Only update sensors and transmit every 2000ms
  if (system_active && (millis() - lastToFUpdate >= 2000)) {
    lastToFUpdate = millis();
    processAndTransmitMatrix(0, tof1, distances1, filtered1, init1, ID_TOF1_BASE);
    processAndTransmitMatrix(1, tof2, distances2, filtered2, init2, ID_TOF2_BASE);
    processAndTransmitMatrix(2, tof3, distances3, filtered3, init3, ID_TOF3_BASE);
  }

  // Print locally at the exact same 2000ms pace
  if (system_active && (millis() - lastDisplayTime >= 2000)) {
    lastDisplayTime = millis();
    
    Serial.println("\n==================== ESP32 SOURCE TRANSMIT DASHBOARD ======================");
    Serial.println("           [Col 0]  [Col 1]  [Col 2]  [Col 3]  [Col 4]  [Col 5]  [Col 6]  [Col 7]");
    
    for (int s = 0; s < 3; s++) {
      Serial.printf("ToF SENSOR %d: ", s + 1);
      for (int c = 0; c < 8; c++) {
        uint16_t distance = local_matrix[s][c];
        if (distance >= 3000 || distance <= 20) {
          Serial.print("  ----   ");
        } else {
          Serial.printf(" %4dmm  ", distance);
        }
      }
      Serial.println();
    }
    Serial.println("============================================================================");
  }
}