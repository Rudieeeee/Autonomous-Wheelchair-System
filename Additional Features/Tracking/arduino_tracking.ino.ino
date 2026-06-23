#define USB_BAUD 115200
#define UWB_BAUD 921600

class UwbAnchorReader {
public:
  UwbAnchorReader(HardwareSerial &serialPort, const char *anchorAddress)
    : port(serialPort), address(anchorAddress) {}

  void begin() {
    port.begin(UWB_BAUD);
  }

  void poll() {
    while (port.available()) {
      char c = port.read();

      if (c == '\n') {
        handleLine(line);
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
  }

private:
  HardwareSerial &port;
  const char *address;
  String line = "";

  void handleLine(String raw) {
    raw.trim();

    int distanceCm = parseDistanceCm(raw);

    if (distanceCm <= 0) {
      return;
    }

    Serial.print("UWB,");
    Serial.print(address);
    Serial.print(",");
    Serial.println(distanceCm);
  }

  int parseDistanceCm(String raw) {
    if (!raw.startsWith("P,")) {
      return -1;
    }

    int comma1 = raw.indexOf(',');
    int comma2 = raw.indexOf(',', comma1 + 1);

    if (comma1 < 0 || comma2 < 0) {
      return -1;
    }

    String distanceText = raw.substring(comma2 + 1);
    distanceText.replace("cm", "");
    distanceText.trim();

    return distanceText.toInt();
  }
};

UwbAnchorReader anchor1(Serial1, "1111");
UwbAnchorReader anchor2(Serial2, "2222");
UwbAnchorReader anchor3(Serial3, "3333");

void setup() {
  Serial.begin(USB_BAUD);

  unsigned long waitEnd = millis() + 3000;
  while (!Serial && millis() < waitEnd) {}

  anchor1.begin();
  anchor2.begin();
  anchor3.begin();

  delay(1000);

  Serial.println("UWB three anchor readout started");
}

void loop() {
  anchor1.poll();
  anchor2.poll();
  anchor3.poll();
}