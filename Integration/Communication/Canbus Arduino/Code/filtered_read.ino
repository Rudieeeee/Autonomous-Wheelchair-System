#include <Arduino_CAN.h>

const uint32_t JOYSTICK_CAN_ID = 0x82000000;
const uint32_t STEP_CAN_ID     = 0x0A040000;
const uint32_t LIGHTS_CAN_ID   = 0x0C000303;
const uint32_t PROFILE_ID      = 0x051;
const int STANDBY_PIN = 7;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);
  delay(10);

  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("CAN init failed");
    while (1);
  }

  Serial.println("Filtered CAN reader started");
}

void loop() {
  if (CAN.available()) {
    CanMsg msg = CAN.read();

    uint32_t id = msg.id;

    if (id == JOYSTICK_CAN_ID) {
      Serial.print("JOYSTICK: ");
    } 
    else if (id == STEP_CAN_ID) {
      Serial.print("STEP: ");
    } 
    else if (id == LIGHTS_CAN_ID) {
      Serial.print("LIGHTS: ");
    } 
    else if (id == PROFILE_ID) {
      Serial.print("PROFILE: ");
    } 
    else {
      return;
    }

    Serial.print("ID=0x");
    Serial.print(id, HEX);
    Serial.print(" DLC=");
    Serial.print(msg.data_length);
    Serial.print(" DATA=");

    for (int i = 0; i < msg.data_length; i++) {
      if (msg.data[i] < 0x10) Serial.print("0");
      Serial.print(msg.data[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }
}