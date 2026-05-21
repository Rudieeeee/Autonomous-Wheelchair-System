#include <Arduino.h>
#include "driver/twai.h"

// XIAO ESP32-S3 pins
#define CAN_TX_PIN GPIO_NUM_43   // D6
#define CAN_RX_PIN GPIO_NUM_44   // D7

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("CAN sender starting on D6/D7...");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN,
    CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("STATUS,twai_driver_install_failed");
    while (1) delay(1000);
  }

  if (twai_start() != ESP_OK) {
    Serial.println("STATUS,twai_start_failed");
    while (1) delay(1000);
  }

  Serial.println("STATUS,can_sender_ready");
}

void loop() {
  twai_message_t message = {};

  message.identifier = 0x123;
  message.extd = 0;  // 0 = standard ID, 1 = extended ID
  message.rtr = 0;
  message.data_length_code = 4;

  message.data[0] = 0x11;
  message.data[1] = 0x22;
  message.data[2] = 0x33;
  message.data[3] = 0x44;

  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(1000));

  if (result == ESP_OK) {
    Serial.println("STATUS,message_sent");
  } else {
    Serial.println("STATUS,message_send_failed");
  }

  delay(500);
}