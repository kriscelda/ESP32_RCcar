#include <CAN.h>

#define ECU_RESPONSE_ID 0x7E8   // ECU response frame ID
#define TESTER_REQUEST_ID 0x7DF // Broadcast OBD2 request

// Supported sensor PIDs
#define PID_SUPPORTED_00       0x00
#define PID_ENGINE_RPM         0x0C
#define PID_VEHICLE_SPEED      0x0D
#define PID_THROTTLE_POSITION  0x11
#define PID_DISTANCE_SINCE_DTC 0x31
#define PID_COOLANT_TEMP       0x05
#define PID_DISTANCE_SINCE_START 0x21

// VIN Info (Mode 09, PID 02)
const char VIN[] = "1HGBH41JXMN109186"; // 17 bytes

bool timerActive = false;
unsigned long waitStartMs = 0;
bool vinInProgress = false;
unsigned long vinLastActivityMs = 0;
const unsigned long VIN_TIMEOUT_MS = 100;  // safety timeout

// ===== REAL DATA TELEMETRY STORAGE =====
uint16_t rpm = 0;       // Controlled via RPi USB
uint8_t speed = 0;      // Controlled via RPi USB
uint8_t coolant = 82;
float throttle = 5.5;
uint16_t distance = 1542;
uint16_t distance_main = 20000;

// Serial parsing variables
String inputString = "";
bool stringComplete = false;

// ===== DTC STORAGE =====
bool dtcActive = true;  // toggle for testing

uint8_t dtcData[] = {
  0x01, 0x71,  // P0171
  0x03, 0x00,  // P0300
  0x04, 0x20   // P0420
};
uint8_t dtcCount = 3;

void setup() {
  // Bump baud rate up to match Python script configuration
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting OBD2 Emulator with live USB input...");

  if (!CAN.begin(1000E3)) {
    Serial.println("CAN init failed");
    while (1);
  }

  Serial.println("CAN init success.");
  inputString.reserve(32);
  delay(100);
}

void loop() {
  // 1. READ LIVE TRUCK DATA FROM RASPBERRY PI
  parseSerialTelemetry();

  // 2. PROCESS INCOMING CAN OBD2 QUERIES
  if (CAN.parsePacket()) {
    uint32_t id = CAN.packetId();
    if (id == TESTER_REQUEST_ID || (id >= 0x7E0 && id <= 0x7E7)) {
      uint8_t len = CAN.read();
      uint8_t mode = CAN.read();
      uint8_t pid = CAN.read();
     
      if (mode == 0x09) {
        if (pid == 0x02){
          vinInProgress = true;
          vinLastActivityMs = millis();
          handleMode09_VIN();
          return;
        }
      }
      if (vinInProgress) {
        return;   // ignore Mode 01 temporarily
      }
      if (mode == 0x03) {
        handleMode03();
      }
      else if (mode == 0x04) {
        handleMode04();
      }
      else if (mode == 0x01) {
        handleMode01(pid);
      }
    }
  }
  
  if (vinInProgress && (millis() - vinLastActivityMs > VIN_TIMEOUT_MS)) {
    vinInProgress = false;
  }
}

// Non-blocking parser for serial telemetry string stream ($RPM,SPEED,THROTTLE\n)
void parseSerialTelemetry() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
    
    if (stringComplete) {
      int startSign = inputString.indexOf('$');
      if (startSign != -1) {
        String cleanData = inputString.substring(startSign + 1);
        cleanData.trim(); // Clean trailing spaces/invisible formatting characters
        
        int firstComma = cleanData.indexOf(',');
        if (firstComma != -1) {
          String rpmStr = cleanData.substring(0, firstComma);
          String restOfData = cleanData.substring(firstComma + 1);
          
          int secondComma = restOfData.indexOf(',');
          if (secondComma != -1) {
            String speedStr = restOfData.substring(0, secondComma);
            String throttleStr = restOfData.substring(secondComma + 1);
            
            rpmStr.trim();
            speedStr.trim();
            throttleStr.trim();

            // Assign parsed data strings safely to telemetry variables
            rpm = rpmStr.toInt();
            speed = speedStr.toInt();
            throttle = throttleStr.toFloat(); 
          } else {
            // Fallback for parsing standard two-value format strings safely
            rpmStr.trim();
            restOfData.trim();
            rpm = rpmStr.toInt();
            speed = restOfData.toInt();
          }
        }
      }
      
      inputString = "";
      stringComplete = false;
    }
  }
}

// ========== MODE 01 HANDLER ==========
void handleMode01(uint8_t pid) {
  switch (pid) {
    case PID_SUPPORTED_00:
      send4Bytes(pid, 0xBE, 0x1F, 0xA8, 0x13);
      break;
    case PID_ENGINE_RPM:
      // Real telemetry inputs processed natively 
      send2Bytes(pid, rpm * 4); // OBD Formula: (A*256 + B) / 4
      break;
    case PID_VEHICLE_SPEED:
      // Real telemetry inputs processed natively
      send1Byte(pid, speed);
      break;
    case PID_COOLANT_TEMP:
      send1Byte(pid, coolant + 40);
      break;
    case PID_THROTTLE_POSITION:
    //put the throttle data 
      send1Byte(pid, (uint8_t)((throttle * 255.0) / 100.0));
      break;
    case PID_DISTANCE_SINCE_DTC:
      send2Bytes(pid, distance);
      break;
    case PID_DISTANCE_SINCE_START:
      send2Bytes(pid, distance_main);
      break;
    default:
      break;
  }
}

void handleMode03() {
  CAN.beginPacket(ECU_RESPONSE_ID);

  if (!dtcActive || dtcCount == 0) {
    // No DTCs
    CAN.write(0x02);
    CAN.write(0x43);
    CAN.write(0x00);
    padZeros(5);
    CAN.endPacket();
    Serial.println("No DTCs");
    return;
  }

  uint8_t totalBytes = 1 + (dtcCount * 2); // 43 + DTC bytes

  CAN.write(totalBytes);
  CAN.write(0x43);

  for (int i = 0; i < dtcCount * 2; i++) {
    CAN.write(dtcData[i]);
  }

  padZeros(8 - (totalBytes + 1));
  CAN.endPacket();

  Serial.println("Sent DTCs");
}

void handleMode04() {
  dtcActive = false;

  CAN.beginPacket(ECU_RESPONSE_ID);
  CAN.write(0x01);
  CAN.write(0x44);  // Mode 04 response
  padZeros(6);
  CAN.endPacket();

  Serial.println("DTCs Cleared");
}

// ========== MODE 09 HANDLER (VIN) ==========

void handleMode09_VIN() {

  // ===== FIRST FRAME =====
  CAN.beginPacket(ECU_RESPONSE_ID);
  CAN.write(0x10);  // First Frame
  CAN.write(0x14);  // Total length = 20 bytes (0x14)
  CAN.write(0x49);  // Mode 09 response
  CAN.write(0x02);  // PID 02 (VIN)
  CAN.write(0x01);  // Frame index
  CAN.write(VIN[0]); CAN.write(VIN[1]); CAN.write(VIN[2]);
  CAN.write(VIN[3]); CAN.write(VIN[4]);
  CAN.endPacket();

  Serial.println("VIN FF sent");

  // ===== WAIT FOR FLOW CONTROL =====
  uint8_t blockSize = 0;
  uint8_t stmin = 5; // default fallback (ms)

  unsigned long start = millis();
  while (millis() - start < 200) {
    if (CAN.parsePacket()) {
      uint32_t id = CAN.packetId();

      if (id == TESTER_REQUEST_ID || (id >= 0x7E0 && id <= 0x7E7)) {

        uint8_t len = CAN.read();
        uint8_t pci = CAN.read();

        if ((pci & 0xF0) == 0x30) {  // Flow Control
          blockSize = CAN.read();
          stmin = CAN.read();

          Serial.print("FC received | BS=");
          Serial.print(blockSize);
          Serial.print(" STmin=");
          Serial.println(stmin);

          break;
        }
      }
    }
  }

  // ===== CONSECUTIVE FRAME 1 =====
  delay(stmin);

  CAN.beginPacket(ECU_RESPONSE_ID);
  CAN.write(0x21);
  CAN.write(VIN[5]); CAN.write(VIN[6]); CAN.write(VIN[7]); CAN.write(VIN[8]);
  CAN.write(VIN[9]); CAN.write(VIN[10]); CAN.write(VIN[11]); CAN.write(VIN[12]);
  CAN.endPacket();

  Serial.println("VIN CF1 sent");

  // ===== CONSECUTIVE FRAME 2 =====
  delay(stmin);

  CAN.beginPacket(ECU_RESPONSE_ID);
  CAN.write(0x22);
  CAN.write(VIN[13]); CAN.write(VIN[14]); CAN.write(VIN[15]); CAN.write(VIN[16]);
  CAN.write(0x00); CAN.write(0x00); CAN.write(0x00); CAN.write(0x00);
  CAN.endPacket();

  Serial.println("VIN CF2 sent");

  Serial.println("VIN multi-frame complete");
}
  


void send1Byte(uint8_t pid, uint8_t A) {
  CAN.beginPacket(ECU_RESPONSE_ID);
  CAN.write(0x03);
  CAN.write(0x41);
  CAN.write(pid);
  CAN.write(A);
  padZeros(4);
  CAN.endPacket();
}

void send2Bytes(uint8_t pid, uint16_t val) {
  CAN.beginPacket(ECU_RESPONSE_ID);
  CAN.write(0x04);
  CAN.write(0x41);
  CAN.write(pid);
  CAN.write(val >> 8);
  CAN.write(val & 0xFF);
  padZeros(3);
  CAN.endPacket();
}

void send4Bytes(uint8_t pid, uint8_t A, uint8_t B, uint8_t C, uint8_t D) {
  CAN.beginPacket(ECU_RESPONSE_ID);
  CAN.write(0x06);
  CAN.write(0x41);
  CAN.write(pid);
  CAN.write(A); CAN.write(B); CAN.write(C); CAN.write(D);
  CAN.write(0x00); // padding
  CAN.endPacket();
}

void padZeros(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    CAN.write(0x00);
  }
}

bool waitMs(unsigned long waitTime) {
  if (!timerActive) {
    waitStartMs = millis();
    timerActive = true;
    return false;           // still waiting
  }

  if (millis() - waitStartMs >= waitTime) {
    timerActive = false;    // done waiting
    return true;
  }

  return false;             // still waiting
}
