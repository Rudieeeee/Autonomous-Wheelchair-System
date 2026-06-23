// Arduino GIGA + EWM550-7G9T10SP one-time configuration sketch
// Configure 1 moving base station + 3 stationary responder tags.
//
// Base station on Serial1
// Tag 1 on Serial2
// Tag 2 on Serial3
// Tag 3 on Serial4
//
// Default module UART: 921600 baud, 8N1

#define UWB_BAUD 921600

const bool RUN_CONFIGURATION = true;

void readResponse(Stream &port, const char *name, uint32_t timeoutMs = 600) {
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (port.available()) {
      char c = port.read();
      Serial.write(c);
      start = millis(); // extend while data is still arriving
    }
  }

  Serial.println();
}

void sendRaw(Stream &port, const char *name, const char *text, uint32_t waitMs = 600) {
  Serial.print("[");
  Serial.print(name);
  Serial.print("] >> ");
  Serial.println(text);

  port.print(text);
  readResponse(port, name, waitMs);
}

void sendCmd(Stream &port, const char *name, const char *cmd, uint32_t waitMs = 600) {
  Serial.print("[");
  Serial.print(name);
  Serial.print("] >> ");
  Serial.println(cmd);

  port.print(cmd);
  port.print("\r\n");

  readResponse(port, name, waitMs);
}

void enterATMode(Stream &port, const char *name) {
  // Manual says send +++ to enter AT mode.
  // Do not add newline for this one.
  sendRaw(port, name, "+++", 1000);
}

void configureBase(Stream &port, const char *name) {
  Serial.println();
  Serial.print("Configuring ");
  Serial.println(name);

  enterATMode(port, name);

  sendCmd(port, name, "AT+VERSION");
  sendCmd(port, name, "AT+ROLE=1");
  sendCmd(port, name, "AT+RESPONDER_NUM=3");
  sendCmd(port, name, "AT+CH=9");
  sendCmd(port, name, "AT+POWER=3");
  sendCmd(port, name, "AT+SRCADDR=0000");
  sendCmd(port, name, "AT+DSTADDR=11112222333300000000");
  sendCmd(port, name, "AT+INTV=200");

  // Settings take effect after reset.
  sendCmd(port, name, "AT+RESET", 1200);
}

void configureTag(Stream &port, const char *name, const char *srcAddr) {
  Serial.println();
  Serial.print("Configuring ");
  Serial.println(name);

  enterATMode(port, name);

  sendCmd(port, name, "AT+VERSION");
  sendCmd(port, name, "AT+ROLE=0");
  sendCmd(port, name, "AT+RESPONDER_NUM=3");
  sendCmd(port, name, "AT+CH=9");
  sendCmd(port, name, "AT+POWER=3");

  char srcCmd[32];
  snprintf(srcCmd, sizeof(srcCmd), "AT+SRCADDR=%s", srcAddr);
  sendCmd(port, name, srcCmd);

  // For a tag, the first 4 hex digits of DSTADDR should be the base address.
  sendCmd(port, name, "AT+DSTADDR=00000000000000000000");
  sendCmd(port, name, "AT+INTV=200");

  // Settings take effect after reset.
  sendCmd(port, name, "AT+RESET", 1200);
}

void setup() {
  Serial.begin(115200);

  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) {
    // wait briefly for Serial Monitor
  }

  Serial1.begin(UWB_BAUD);
  Serial2.begin(UWB_BAUD);
  Serial3.begin(UWB_BAUD);
  Serial4.begin(UWB_BAUD);

  delay(1000);

  Serial.println("EWM550 1-base + 3-tag configuration sketch");

  if (!RUN_CONFIGURATION) {
    Serial.println("RUN_CONFIGURATION is false; not configuring modules.");
    return;
  }

  configureBase(Serial1, "BASE");

  configureTag(Serial2, "TAG1", "1111");
  configureTag(Serial3, "TAG2", "2222");
  configureTag(Serial4, "TAG3", "3333");

  Serial.println();
  Serial.println("Configuration done.");
  Serial.println("Now upload your normal ranging/tracking sketch.");
}

void loop() {
}