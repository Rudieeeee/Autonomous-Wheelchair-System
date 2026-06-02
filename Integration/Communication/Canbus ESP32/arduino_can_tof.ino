#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

const uint32_t ID_SYSTEM_CTRL = 0x010;
const uint32_t ID_MASTER_PING = 0x400;

bool system_active = false;
unsigned long lastPingTime = 0;
unsigned long lastDisplayTime = 0;
uint8_t pingCount = 0;

unsigned long node_last_seen[4] = {0, 0, 0, 0}; 
uint16_t live_matrix[8][8]; 

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 8; c++) {
      live_matrix[s][c] = 4000;
    }
  }

  if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("\n=======================================================");
    Serial.println("GIGA MASTER MATRIX NETWORK TERMINAL ONLINE.");
    Serial.println("-------------------------------------------------------");
    Serial.println(" >> Action: Type 'S' and hit Enter to START system.");
    Serial.println(" >> Action: Type 'H' and hit Enter to HALT system.");
    Serial.println("=======================================================");
  } else {
    Serial.println("MCP2515 Hardware Initialization Failed!");
    while (1) delay(100);
  }
  CAN.setMode(MCP_NORMAL);
}

void loop() {
  if (Serial.available() > 0) {
    char inputCmd = toupper(Serial.read());
    if (inputCmd == 'S' && !system_active) {
      system_active = true;
      uint8_t ctrl_payload[1] = { 1 };
      CAN.sendMsgBuf(ID_SYSTEM_CTRL, 0, 1, ctrl_payload);
      Serial.println("\n[COMMAND] >>> SYSTEM ACTIVATED. PINGING BUS... <<<");
    } 
    else if (inputCmd == 'H' && system_active) {
      system_active = false;
      uint8_t ctrl_payload[1] = { 0 };
      CAN.sendMsgBuf(ID_SYSTEM_CTRL, 0, 1, ctrl_payload);
      Serial.println("\n[COMMAND] >>> SYSTEM HALTED. NETWORK IN SLEEP STATE. <<<");
    }
  }

  if (system_active && (millis() - lastPingTime >= 500)) {
    lastPingTime = millis();
    pingCount++;
    uint8_t ping_payload[1] = { pingCount };
    CAN.sendMsgBuf(ID_MASTER_PING, 0, 1, ping_payload);
  }

  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId >= 0x401 && rxId <= 0x403) {
      uint8_t nodeIdx = rxId - 0x400;
      if (nodeIdx <= 3) {
        node_last_seen[nodeIdx] = millis();
      }
    }
    else if (rxId >= 0x110 && rxId <= 0x181) {
      uint8_t sensorNum = ((rxId - 0x110) / 0x10) + 1;
      
      for (int i = 0; i < 4; i++) {
        uint16_t mergedPacket = (rxBuf[i * 2 + 1] << 8) | rxBuf[i * 2];
        uint16_t parsedDist = mergedPacket >> 3;
        uint8_t parsedColumn = mergedPacket & 0x07;

        if (sensorNum >= 1 && sensorNum <= 8 && parsedColumn < 8) {
          live_matrix[sensorNum - 1][parsedColumn] = parsedDist;
        }
      }
    }
  }

  // Prints the stable, bus-frozen values every 2000ms
  if (system_active && (millis() - lastDisplayTime >= 2000)) {
    lastDisplayTime = millis();
    
    Serial.println("\n==================== GIGA MASTER RECEIVE DASHBOARD ========================");
    Serial.print("BUS STATUS: ");
    for (int n = 1; n <= 3; n++) {
      Serial.print("[Node "); Serial.print(n); Serial.print(": ");
      if (millis() - node_last_seen[n] < 2500) { // Bumped window slightly to match slower transmissions
        Serial.print("ONLINE]  ");
      } else {
        Serial.print("OFFLINE] ");
      }
    }
    Serial.println("\n----------------------------------------------------------------------------");
    Serial.println("           [Col 0]  [Col 1]  [Col 2]  [Col 3]  [Col 4]  [Col 5]  [Col 6]  [Col 7]");
    
    for (int s = 0; s < 3; s++) {
      Serial.print("ToF SENSOR ");
      Serial.print(s + 1);
      Serial.print(": ");
      
      for (int c = 0; c < 8; c++) {
        uint16_t distance = live_matrix[s][c];
        
        if (distance >= 3000 || distance <= 20) {
          Serial.print("  ----   "); 
        } else {
          char formatBuf[16];
          sprintf(formatBuf, " %4dmm  ", distance);
          Serial.print(formatBuf);
        }
      }
      Serial.println();
    }
    Serial.println("============================================================================");
  }
}