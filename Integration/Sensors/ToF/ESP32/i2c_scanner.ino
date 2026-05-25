#include <Wire.h>

#define I2C_SDA 5
#define I2C_SCL 6

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("I2C scanner started");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found I2C device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  Serial.println("Scan done");
}

void loop() {}