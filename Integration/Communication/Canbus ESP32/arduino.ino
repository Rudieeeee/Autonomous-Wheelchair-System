#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

const uint32_t ID_SYSTEM_CTRL = 0x010;
const uint32_t ID_MASTER_PING = 0x400;
const uint32_t ID_CALIBRATE   = 0x011;
const uint32_t ID_CALIB_STAT  = 0x012;

uint8_t system_mode = 0;
unsigned long lastPingTime = 0;
unsigned long lastDisplayTime = 0;
uint8_t pingCount = 0;

unsigned long node_last_seen[4] = {0, 0, 0, 0};
bool tof_alive[8] = {false}; // Array for all 8 ToFs (Index 0-7)

// live_matrix mapping:
// 0: T1, 1: T2, 2: T3 (Node 1 - Front)
// 3: T4, 4: T5, 5: T6 (Node 3 - Front)
// 6: T7, 7: T8        (Node 2 - Rear)
uint16_t live_matrix[8][8]; 
uint32_t framesReceived = 0;

void printMenu() {
  Serial.println("\n=======================================================");
  Serial.println("  GIGA MASTER: FULL VEHICLE TRACKING ONLINE");
  Serial.println("=======================================================");
  Serial.println(" COMMANDS:");
  Serial.println("  [0] = HALT: Sleep network");
  Serial.println("  [1] = MODE 1: Drive (Standard Min-Distance)");
  Serial.println("  [2] = MODE 2: Align Mode (Center Row)");
  Serial.println("  [3] = MODE 3: Floor Test (Bottom Rows)");
  Serial.println("  [D] = MODE D: Dynamic Baseline Drive");
  Serial.println("  [C] = CALIBRATE: Generate 5s Floor Map");
  Serial.println("=======================================================");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  SPI.begin();

  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 8; c++) live_matrix[s][c] = 4000;
  }

  if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
    printMenu();
  } else {
    Serial.println("MCP2515 Init Failed! Check wiring.");
    while (1) delay(100);
  }
  CAN.setMode(MCP_NORMAL);
}

void printDashboard(bool isCalibrationResult) {
  Serial.println("\n----------------------------------------------------------------------------");
  
  if (isCalibrationResult) {
    Serial.println(" >>> [ CALIBRATION COMPLETE: BASELINE MAP RESULTS ] <<<");
  } else {
    Serial.print("NODES: [N1:"); Serial.print((millis() - node_last_seen[1] < 2500) ? "ON] " : "OFF]");
    Serial.print("[N2:"); Serial.print((millis() - node_last_seen[2] < 2500) ? "ON] " : "OFF]");
    Serial.print("[N3:"); Serial.println((millis() - node_last_seen[3] < 2500) ? "ON] " : "OFF]");
    
    Serial.print("TOFS:  [N1: ");
    for(int i=0;i<3;i++) Serial.print(tof_alive[i] ? "OK " : "X ");
    Serial.print("] [N3: ");
    for(int i=3;i<6;i++) Serial.print(tof_alive[i] ? "OK " : "X ");
    Serial.print("] [N2(Rear): ");
    for(int i=6;i<8;i++) Serial.print(tof_alive[i] ? "OK " : "X ");
    Serial.println("]");
  }

  Serial.println("\n--- FRONT SENSORS (Nodes 1 & 3) ---");
  for (int s = 0; s < 6; s++) {
    Serial.print("ToF "); Serial.print(s + 1); Serial.print(": ");
    if (!tof_alive[s] && !isCalibrationResult) {
      Serial.println("  [ DISCONNECTED ]");
    } else {
      for (int c = 0; c < 8; c++) {
        uint16_t d = live_matrix[s][c];
        if (d == 0 || d >= 3000) Serial.print("  ----   "); 
        else { char formatBuf[16]; sprintf(formatBuf, " %4dmm  ", d); Serial.print(formatBuf); }
      }
      Serial.println();
    }
  }

  Serial.println("\n--- REAR SENSORS (Node 2) ---");
  for (int s = 6; s < 8; s++) {
    Serial.print("ToF "); Serial.print(s + 1); Serial.print(": ");
    if (!tof_alive[s] && !isCalibrationResult) {
      Serial.println("  [ DISCONNECTED ]");
    } else {
      for (int c = 0; c < 8; c++) {
        uint16_t d = live_matrix[s][c];
        if (d == 0 || d >= 3000) Serial.print("  ----   "); 
        else { char formatBuf[16]; sprintf(formatBuf, " %4dmm  ", d); Serial.print(formatBuf); }
      }
      Serial.println();
    }
  }
}

void loop() {
  if (Serial.available() > 0) {
    char inputCmd = toupper(Serial.read());
    bool validMode = false;

    if (inputCmd >= '0' && inputCmd <= '3') { system_mode = inputCmd - '0'; validMode = true; } 
    else if (inputCmd == 'D') { system_mode = 4; validMode = true; }

    if (validMode) {
      uint8_t payload[1] = { system_mode };
      CAN.sendMsgBuf(ID_SYSTEM_CTRL, 0, 1, payload);
      if (system_mode == 0) { Serial.println("\n[COMMAND] >>> NETWORK HALTED <<<"); printMenu(); } 
      else Serial.println("\n[COMMAND] >>> MODE SWITCHED <<<");
    }

    if (inputCmd == 'C') {
      uint8_t payload[1] = {1};
      CAN.sendMsgBuf(ID_CALIBRATE, 0, 1, payload);
      Serial.println("\n[COMMAND] >>> CALIBRATION STARTED (Keep area clear for 5s) <<<");
    }
  }

  if (system_mode > 0 && (millis() - lastPingTime >= 500)) {
    lastPingTime = millis();
    pingCount++;
    uint8_t ping_payload[1] = { pingCount };
    CAN.sendMsgBuf(ID_MASTER_PING, 0, 1, ping_payload);
  }

  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId; unsigned char len = 0; unsigned char rxBuf[8];
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId == ID_CALIB_STAT && len > 0 && rxBuf[0] == 2) {
      printDashboard(true); // Force print the received baseline map immediately
    }
    else if (rxId >= 0x401 && rxId <= 0x403) {
      uint8_t nodeIdx = rxId - 0x400;
      node_last_seen[nodeIdx] = millis();

      if (len >= 4) {
        if (rxId == 0x401) { tof_alive[0] = rxBuf[1]; tof_alive[1] = rxBuf[2]; tof_alive[2] = rxBuf[3]; }
        if (rxId == 0x403) { tof_alive[3] = rxBuf[1]; tof_alive[4] = rxBuf[2]; tof_alive[5] = rxBuf[3]; }
        if (rxId == 0x402) { tof_alive[6] = rxBuf[1]; tof_alive[7] = rxBuf[2]; }
      }
    }
    else if (rxId >= 0x110 && rxId <= 0x181) {
      uint8_t base_index = (rxId - 0x110) / 0x10;
      if (base_index < 8) {
        for (int i = 0; i < 4; i++) {
          uint16_t mergedPacket = (rxBuf[i * 2 + 1] << 8) | rxBuf[i * 2];
          uint16_t parsedDist = mergedPacket >> 3;
          uint8_t parsedColumn = mergedPacket & 0x07;
          if (parsedColumn < 8) live_matrix[base_index][parsedColumn] = parsedDist;
        }
      }
    }
  }

  if (system_mode > 0 && (millis() - lastDisplayTime >= 1000)) {
    lastDisplayTime = millis();
    printDashboard(false);
  }
}