#include <Arduino.h>
#include "driver/twai.h"

// XIAO ESP32-S3 pins
#define CAN_TX_PIN GPIO_NUM_43   // D6
#define CAN_RX_PIN GPIO_NUM_44   // D7

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("CAN receiver starting on D6/D7...");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN,
    CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("STATUS,twai_driver_install_failed");
    while (1) delay(1000);
  }

  if (twai_start() != ESP_OK) {
    Serial.println("STATUS,twai_start_failed");
    while (1) delay(1000);
  }

  Serial.println("STATUS,can_receiver_ready");
}

void loop() {
  twai_message_t message;

  esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(1000));

  if (result == ESP_OK) {
    Serial.print("CAN,ID=0x");
    Serial.print(message.identifier, HEX);

    if (message.extd) {
      Serial.print(",EXT");
    } else {
      Serial.print(",STD");
    }

    Serial.print(",DLC=");
    Serial.print(message.data_length_code);

    Serial.print(",DATA=");
    for (int i = 0; i < message.data_length_code; i++) {
      if (message.data[i] < 0x10) Serial.print("0");
      Serial.print(message.data[i], HEX);
      if (i < message.data_length_code - 1) Serial.print(" ");
    }

    Serial.println();
  } else {
    Serial.println("STATUS,no_message");
  }
}