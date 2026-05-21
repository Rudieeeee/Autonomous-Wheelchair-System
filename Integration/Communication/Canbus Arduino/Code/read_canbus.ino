#include <Arduino_CAN.h>

const int STANDBY_PIN = 7;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting CAN reader...");

  // Enable CAN transceiver / take it out of standby
  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);

  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("CAN init failed");
    while (1);
  }

  Serial.println("CAN reader started at 125 kbps");
}

void loop() {
  if (CAN.available()) {
    CanMsg msg = CAN.read();

    Serial.print("ID: 0x");
    Serial.print(msg.id, HEX);

    if (msg.isExtendedId()) {
      Serial.print(" EXT");
    } else {
      Serial.print(" STD");
    }

    Serial.print(" DLC: ");
    Serial.print(msg.data_length);

    Serial.print(" DATA: ");
    for (int i = 0; i < msg.data_length; i++) {
      if (msg.data[i] < 0x10) Serial.print("0");
      Serial.print(msg.data[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }
}