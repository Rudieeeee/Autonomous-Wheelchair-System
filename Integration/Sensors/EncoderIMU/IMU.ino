#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("BOOT");

  Wire.begin();
  Wire.setClock(100000);

  Serial.println("Starting BNO055...");

  if (!bno.begin()) {
    Serial.println("BNO055 not detected by Adafruit library");

    while (true) {
      Serial.println("Still alive, but no BNO055");
      delay(1000);
    }
  }

  Serial.println("BNO055 detected");
  bno.setExtCrystalUse(true);
}

void loop() {
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    sensors_event_t event;
    bno.getEvent(&event);

    Serial.print("Yaw=");
    Serial.print(event.orientation.x);
    Serial.print(" Pitch=");
    Serial.print(event.orientation.y);
    Serial.print(" Roll=");
    Serial.println(event.orientation.z);
  }
}