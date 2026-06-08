#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!bno08x.begin_I2C()) {
    Serial.println("Error: BNO085 not detected check wiring!");
    while (1);
  }

  // Set reports to register at 10000 microseconds interval (100Hz)
  if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) Serial.println("Could not enable accelerometer");
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) Serial.println("Could not enable gyroscope");
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000)) Serial.println("Could not enable magnetometer");

  delay(1000);
}

void loop() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_ACCELEROMETER, 10000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // Collect incoming metrics based on event type
    static float ax, ay, az, gx, gy, gz, mx, my, mz;
    
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        gx = sensorValue.un.gyroscope.x;
        gy = sensorValue.un.gyroscope.y;
        gz = sensorValue.un.gyroscope.z;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mx = sensorValue.un.magneticField.x;
        my = sensorValue.un.magneticField.y;
        mz = sensorValue.un.magneticField.z;
        
        // Print packet sequentially on the magnetometer update cadence
        Serial.print(millis()); Serial.print(",");
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.print(gz); Serial.print(",");
        Serial.print(mx); Serial.print(",");
        Serial.print(my); Serial.print(",");
        Serial.println(mz);
        break;
    }
  }
}