#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_43
#define CAN_RX_PIN GPIO_NUM_44

// CAN ID Map
#define ID_SYSTEM_CTRL   0x010
#define ID_MASTER_PING   0x400
#define NODE_ID          0x401

bool system_active = false;

void setup() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

void loop() {
  twai_message_t rx_msg;
  
  // Continuous low-overhead network listening
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(5)) == ESP_OK) {
    
    // 1. Intercept System Activation Commands (Highest Priority)
    if (rx_msg.identifier == ID_SYSTEM_CTRL && rx_msg.data_length_code > 0) {
      system_active = (rx_msg.data[0] == 1);
    }
    
    // 2. Respond to Background Heartbeats ONLY if system is awake
    else if (system_active && rx_msg.identifier == ID_MASTER_PING) {
      twai_message_t tx_msg = rx_msg;
      tx_msg.identifier = NODE_ID;
      twai_transmit(&tx_msg, pdMS_TO_TICKS(50)); // Allow time to settle contention
    }
  }
}