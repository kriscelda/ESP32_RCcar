#include <Servo.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <TinyGPS++.h>




// ================= WIFI =================
const char* ssid = "steer";
const char* password = "pass1234";


WebSocketsServer webSocket = WebSocketsServer(81);


// ================= GPS (Using Serial2) =================
#define GPS_RX 17
#define GPS_TX 18
#define GPS_BAUD 9600
TinyGPSPlus gps;


// ================= MOTOR / SERVO =================
#define MOTOR_ENA 16
#define MOTOR_IN1 1   // CHANGED from 1 to avoid WiFi-blocking strapping pin
#define MOTOR_IN2 2   // CHANGED from 2 to avoid WiFi-blocking strapping pin
#define LED_PIN 14



static const int servoPin = 5;
const int frequency = 200;


Servo servo1;


// ================= HELPER: JSON Parse =================
int getValue(String data, String key) {
  int index = data.indexOf(key);
  if (index == -1) return 0;
  int start = data.indexOf(":", index) + 1;
  int end = data.indexOf(",", start);
  if (end == -1) end = data.indexOf("}", start);
  return data.substring(start, end).toInt();
}




void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, LOW);
      analogWrite(MOTOR_ENA, 0);
      break;




    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u connected\n", num);
      Serial.printf("Connected Clients: %d\n", webSocket.connectedClients());
      break;


    case WStype_TEXT: {
      String msg = String((char*)payload);

      Serial.println();
      Serial.println("========== WS RECEIVED ==========");
      Serial.println(msg);
      Serial.println("=================================");

      int steer  = getValue(msg, "steer");
      int speed  = getValue(msg, "throttle");
      int brake  = getValue(msg, "brake");
      int dir    = getValue(msg, "dir");
      int button = getValue(msg, "button");
     
      int firstspeed = map(speed, 0, 100, 100, 255);
      int firstbrake = map(brake, 0, 80, 0, 180);
      int finalspeed = firstspeed - firstbrake;
      Serial.println("final speed:" + finalspeed);
      if (finalspeed < 0) finalspeed = 0; // Prevent negative PWM values


      // --- EXECUTE CONTROLS ---
      if (steer >= 0 && steer <= 180) {
        servo1.write(steer);
        Serial.printf("Servo Command: %d\n", steer);
      }


      String gearStr = "NEUTRAL";
      if (dir == 1) { // Forward
        Serial.printf(
        "DIR=%d SPEED=%d FINALPWM=%d\n",
        dir,
        speed,
        finalspeed
      );
        gearStr = "FORWARD (D)";
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_ENA, finalspeed);
      } else if (dir == 2) { // Reverse
        gearStr = "REVERSE (R)";
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        analogWrite(MOTOR_ENA, finalspeed);
      } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_ENA, 0);
      }
      digitalWrite(LED_PIN, button > 0 ? HIGH : LOW);




      // --- PRINT CONTROLLER DASHBOARD TO SERIAL ---
      Serial.println("\n--- CONTROLLER INPUTS ---");
      Serial.printf("Gear State : %s\n", gearStr.c_str());
      Serial.printf("Steering   : %d°\n", steer);
      Serial.printf("Pedal (Gas): %d%%\n", speed);
      Serial.printf("Pedal (Brk): %d%%\n", brake);
      Serial.printf("Motor PWM  : %d / 255\n", finalspeed);
      Serial.printf("Aux Button : %s\n", (button > 0) ? "ON" : "OFF");
      Serial.println("-------------------------");
     
      break;
    }
  }
}




void setup() {
  Serial.begin(115200);
 
  // Initialize Serial2 for GPS
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);




  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);


  servo1.attach(servoPin, Servo::CHANNEL_NOT_ATTACHED, 45, 145, 500, 2400, frequency);


  WiFi.mode(WIFI_STA); // Force station mode
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());




  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}




void loop() {
  webSocket.loop();


  // --- GPS LOGIC ---
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }


  // Periodic Broadcast (Every 500ms for smooth dashboard updates)
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast > 500) {
    if (gps.location.isValid()) {
      String tele = "{\"lat\":" + String(gps.location.lat(), 6) +
                    ",\"lng\":" + String(gps.location.lng(), 6) +
                    ",\"kmh\":" + String(gps.speed.kmph(), 1) + "}";
      webSocket.broadcastTXT(tele);
    }
    lastBroadcast = millis();
  }
}
