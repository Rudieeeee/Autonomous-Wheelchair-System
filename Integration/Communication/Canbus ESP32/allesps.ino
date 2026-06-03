#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "driver/twai.h"
#include "DFRobot_MatrixLidar.h"

// ================= NODE CONFIGURATION =================
// CHANGE THIS NUMBER TO 1, 2, OR 3 BEFORE FLASHING EACH ESP32!
#define ACTIVE_NODE 1

// ================= CAN =================
#define CAN_TX_PIN GPIO_NUM_43
#define CAN_RX_PIN GPIO_NUM_44

#define ID_SYSTEM_CTRL 0x010
#define ID_MASTER_PING 0x400
#define ID_CALIBRATE   0x011
#define ID_CALIB_STAT  0x012

// Auto-configure IDs based on which node this is
#if ACTIVE_NODE == 1
  #define NODE_ID 0x401
  #define ID_TOF_A 0x110
  #define ID_TOF_B 0x120
  #define ID_TOF_C 0x130
  #define NUM_TOFS 3
#elif ACTIVE_NODE == 2
  #define NODE_ID 0x402
  #define ID_TOF_A 0x170 // Rear ToFs have lower priority IDs
  #define ID_TOF_B 0x180
  #define ID_TOF_C 0x000 // Unused
  #define NUM_TOFS 2
#elif ACTIVE_NODE == 3
  #define NODE_ID 0x403
  #define ID_TOF_A 0x140
  #define ID_TOF_B 0x150
  #define ID_TOF_C 0x160
  #define NUM_TOFS 3
#endif

// ================= I2C =================
#define I2C_SDA 5
#define I2C_SCL 6

DFRobot_MatrixLidar_I2C tofA(0x31, &Wire);
DFRobot_MatrixLidar_I2C tofB(0x32, &Wire);
DFRobot_MatrixLidar_I2C tofC(0x33, &Wire);

const float vertical_angles[8] = {26.25, 18.75, 11.25, 3.75, -3.75, -11.25, -18.75, -26.25};
const float ALPHA = 0.9;
const uint16_t FLOOR_MARGIN = 150;

uint8_t system_mode = 0; 
unsigned long lastUpdate = 0;

uint16_t d1[64], d2[64], d3[64];
float f1[64], f2[64], f3[64];
bool init1=false, init2=false, init3=false;
bool sA_ok = false, sB_ok = false, sC_ok = false;

uint16_t b1[64], b2[64], b3[64];
bool calibrated = false;
bool calibrating = false;

uint32_t s1[64], s2[64], s3[64];
uint16_t calib_samples = 0;
unsigned long calib_start_time = 0;

bool checkI2C(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

void transmitBaselineMap(uint16_t baseline[64], uint16_t can_base_id) {
  uint16_t column_packets[8];
  for (uint8_t col = 0; col < 8; col++) {
    uint16_t min_base = 4000;
    for (uint8_t row = 0; row < 8; row++) {
      if (baseline[row*8+col] > 20 && baseline[row*8+col] < min_base) {
        min_base = baseline[row*8+col];
      }
    }
    column_packets[col] = (min_base << 3) | (col & 0x07);
  }
  twai_message_t tx_msg;
  tx_msg.extd = 0; tx_msg.rtr = 0; tx_msg.data_length_code = 8;
  
  tx_msg.identifier = can_base_id;
  for (int i=0; i<4; i++) { tx_msg.data[i*2] = column_packets[i]&0xFF; tx_msg.data[i*2+1] = (column_packets[i]>>8)&0xFF; }
  twai_transmit(&tx_msg, pdMS_TO_TICKS(5)); delayMicroseconds(200);

  tx_msg.identifier = can_base_id + 1;
  for (int i=0; i<4; i++) { tx_msg.data[i*2] = column_packets[i+4]&0xFF; tx_msg.data[i*2+1] = (column_packets[i+4]>>8)&0xFF; }
  twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
}

void processAndTransmitMatrix(DFRobot_MatrixLidar_I2C& tof, uint16_t distances[64], float filtered[64], bool& initialized, uint16_t can_base_id, uint16_t baseline[64]) {
  if (tof.getAllData(distances) != 0) return;

  uint16_t column_packets[8];
  for (uint8_t col = 0; col < 8; col++) {
    uint16_t target_val = 4000;

    for (uint8_t row = 0; row < 8; row++) {
      int i = row * 8 + col;
      float p = distances[i] == 0 ? 4000.0 : distances[i] * cos(vertical_angles[row] * PI / 180.0);
      if (!initialized) filtered[i] = p;
      filtered[i] = ALPHA * p + (1.0f - ALPHA) * filtered[i];
    }

    if (system_mode == 2) target_val = (uint16_t)filtered[4 * 8 + col]; 
    else if (system_mode == 3) {
      uint16_t min_dist = 65535;
      for (uint8_t r = 6; r < 8; r++) { if ((uint16_t)filtered[r*8+col] > 20 && (uint16_t)filtered[r*8+col] < min_dist) min_dist = (uint16_t)filtered[r*8+col]; }
      if (min_dist < 3500) target_val = min_dist;
    }
    else if (system_mode == 4) {
      if (calibrated) {
        uint16_t min_dist = 65535;
        for (uint8_t r = 0; r < 8; r++) {
          uint16_t m = (uint16_t)filtered[r*8+col];
          uint16_t b = baseline[r*8+col];
          if (abs((int)m - (int)b) <= FLOOR_MARGIN) continue; 
          uint16_t val = (m < b) ? m : b;
          if (val > 20 && val < min_dist) min_dist = val;
        }
        if (min_dist < 3500) target_val = min_dist;
      }
    }
    else {
      uint16_t min_dist = 65535;
      for (uint8_t r = 0; r < 8; r++) { if ((uint16_t)filtered[r*8+col] > 20 && (uint16_t)filtered[r*8+col] < min_dist) min_dist = (uint16_t)filtered[r*8+col]; }
      if (min_dist < 3500) target_val = min_dist;
    }
    column_packets[col] = (target_val << 3) | (col & 0x07);
  }
  initialized = true;

  twai_message_t tx_msg; tx_msg.extd = 0; tx_msg.rtr = 0; tx_msg.data_length_code = 8;
  tx_msg.identifier = can_base_id;
  for (int i=0; i<4; i++) { tx_msg.data[i*2] = column_packets[i]&0xFF; tx_msg.data[i*2+1] = (column_packets[i]>>8)&0xFF; }
  twai_transmit(&tx_msg, pdMS_TO_TICKS(5)); delayMicroseconds(200);

  tx_msg.identifier = can_base_id + 1;
  for (int i=0; i<4; i++) { tx_msg.data[i*2] = column_packets[i+4]&0xFF; tx_msg.data[i*2+1] = (column_packets[i+4]>>8)&0xFF; }
  twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
}

void setup() {
  Serial.begin(115200);
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 20; 
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 
  Wire.setTimeout(10); 
  
  if (checkI2C(0x31) && tofA.begin() == 0) sA_ok = true;
  if (checkI2C(0x32) && tofB.begin() == 0) sB_ok = true;
  if (NUM_TOFS == 3 && checkI2C(0x33) && tofC.begin() == 0) sC_ok = true;
}

void loop() {
  twai_message_t rx_msg;
  while (twai_receive(&rx_msg, pdMS_TO_TICKS(0)) == ESP_OK) {
    if (rx_msg.identifier == ID_SYSTEM_CTRL) system_mode = rx_msg.data[0];
    else if (rx_msg.identifier == ID_CALIBRATE) {
      memset(s1,0,sizeof(s1)); memset(s2,0,sizeof(s2)); memset(s3,0,sizeof(s3));
      calib_samples = 0; calib_start_time = millis(); calibrating = true;
    }
    else if (system_mode > 0 && rx_msg.identifier == ID_MASTER_PING) {
      twai_message_t tx_msg = rx_msg; tx_msg.identifier = NODE_ID;
      tx_msg.data_length_code = 4; tx_msg.data[0] = system_mode;
      tx_msg.data[1] = sA_ok ? 1 : 0; tx_msg.data[2] = sB_ok ? 1 : 0; tx_msg.data[3] = sC_ok ? 1 : 0; 
      twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
    }
  }

  if ((system_mode > 0 || calibrating) && (millis() - lastUpdate >= 60)) {
    lastUpdate = millis();
    
    if (calibrating) {
      if (sA_ok && tofA.getAllData(d1) == 0) { for(int i=0; i<64; i++) s1[i] += (d1[i]==0?4000:(uint32_t)(d1[i]*cos(vertical_angles[i/8]*PI/180.0))); }
      if (sB_ok && tofB.getAllData(d2) == 0) { for(int i=0; i<64; i++) s2[i] += (d2[i]==0?4000:(uint32_t)(d2[i]*cos(vertical_angles[i/8]*PI/180.0))); }
      if (NUM_TOFS == 3 && sC_ok && tofC.getAllData(d3) == 0) { for(int i=0; i<64; i++) s3[i] += (d3[i]==0?4000:(uint32_t)(d3[i]*cos(vertical_angles[i/8]*PI/180.0))); }
      calib_samples++;

      if (millis() - calib_start_time >= 5000) {
        if (calib_samples > 0) {
          for(int i=0; i<64; i++) { b1[i]=s1[i]/calib_samples; b2[i]=s2[i]/calib_samples; b3[i]=s3[i]/calib_samples; }
          calibrated = true;
          
          // Send baseline data to Master right before "Done" signal
          if (sA_ok) transmitBaselineMap(b1, ID_TOF_A);
          if (sB_ok) transmitBaselineMap(b2, ID_TOF_B);
          if (NUM_TOFS == 3 && sC_ok) transmitBaselineMap(b3, ID_TOF_C);
        }
        calibrating = false;
        twai_message_t done_msg; done_msg.identifier = ID_CALIB_STAT; done_msg.extd = 0; done_msg.rtr = 0; done_msg.data_length_code = 1; done_msg.data[0] = 2;
        twai_transmit(&done_msg, pdMS_TO_TICKS(5));
      }
    }
    else {
      if (sA_ok) processAndTransmitMatrix(tofA, d1, f1, init1, ID_TOF_A, b1);
      if (sB_ok) processAndTransmitMatrix(tofB, d2, f2, init2, ID_TOF_B, b2);
      if (NUM_TOFS == 3 && sC_ok) processAndTransmitMatrix(tofC, d3, f3, init3, ID_TOF_C, b3);
    }
  }
}