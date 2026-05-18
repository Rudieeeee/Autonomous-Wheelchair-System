#include <Arduino.h>
#include <Arduino_CAN.h>
#include <Serial.h>


const uint32_t JOYSTICK_CAN_ID = 0x02000400;

#define STANDBY_PIN 7


void setup() {
  Serial.begin(115200);
  

  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);

  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("CAN init failed");
    while (1);
  }

  // Update this line to avoid confusion:
  Serial.println("Ready. Reading of the CAN message is about to start yo");

}


void loop() {
   
    if (!CAN.available()){
        Serial.println("No CAN BUS detected");
        return;
    }

    CanMsg packet = CAN.read();
    if ((packet.id & 0xFFFF0FFF) == CanExtendedId(JOYSTICK_CAN_ID)){

            Serial.print("X coordinate: ");
            Serial.print((int8_t)packet.data[0]);

            Serial.print(", Y coordinate: ");
            Serial.println((int8_t)packet.data[1]);

        }
   
}