#include <WiFi.h>
#include <WebSocketsServer.h>
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

// Wi-Fi credentials
const char* ssid = "iqoo";
const char* password = "........";

// WebSocket server on port 80
WebSocketsServer webSocket = WebSocketsServer(8080);

// Motor pin definitions
// Motor A
const int enA = 14;
const int in1 = 27;
const int in2 = 26;
// Motor B
const int enB = 32;
const int in3 = 25;
const int in4 = 33;

// Control variables
bool moving = false;
unsigned long moveStartTime = 0;
const unsigned long timeoutDuration = 1000; // 1 second timeout
bool usingBluetooth = false;
bool usingWebSocket = false;

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_TEXT: {
      Serial.printf("[%u] Received text: %s\n", num, payload);
      
      // Process the received command
      if (strcmp((char *)payload, "FORWARD") == 0) {
        moveForward();
      } else if (strcmp((char *)payload, "RIGHT") == 0) {
        moveRight();
      } else if (strcmp((char *)payload, "LEFT") == 0) {
        moveLeft();
      } else if (strcmp((char *)payload, "BACKWARD") == 0) {
        moveBackward();
      } else if (strcmp((char *)payload, "STOP") == 0) {
        stopMotors();
        moving = false;
        break;
      }
      
      moving = true;
      moveStartTime = millis(); // Reset the timeout
      usingWebSocket = true;
      usingBluetooth = false;
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("USV Controller Starting...");

  // Set motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Set initial motor speeds to zero
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  // Initialize Dabble Bluetooth
  Serial.println("Initializing Dabble BLE...");
  Dabble.begin("USV");
  Serial.println("Dabble initialized!");

  // Connect to Wi-Fi
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // Print IP address
  
  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("WebSocket server started");
}

void loop() {
  // Process WebSocket events if available
  webSocket.loop();
  
  // Process Bluetooth inputs if not using WebSocket
  if (!usingWebSocket) {
    Dabble.processInput();
    
    // Check GamePad inputs
    if (GamePad.isUpPressed()) {
      moveForward();
      usingBluetooth = true;
      moving = true;
      moveStartTime = millis();
    }
    else if (GamePad.isDownPressed()) {
      moveBackward();
      usingBluetooth = true;
      moving = true;
      moveStartTime = millis();
    }
    else if (GamePad.isRightPressed()) {
      moveRight();
      usingBluetooth = true;
      moving = true;
      moveStartTime = millis();
    }
    else if (GamePad.isLeftPressed()) {
      moveLeft();
      usingBluetooth = true;
      moving = true;
      moveStartTime = millis();
    }
    else if (usingBluetooth) {
      stopMotors();
      usingBluetooth = false;
      moving = false;
    }
  }
  
  // Check for timeout - only apply to WebSocket control
  if (!usingBluetooth && moving && millis() - moveStartTime >= timeoutDuration) {
    stopMotors();
    moving = false;
    usingWebSocket = false;
  }
}

void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255); // Set motor speed to maximum
  analogWrite(enB, 255); // Set motor speed to maximum
  Serial.println("Moving forward");
}

void moveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255); // Set motor speed to maximum
  analogWrite(enB, 255); // Set motor speed to maximum
  Serial.println("Moving backward");
}

void moveRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255); // Set motor speed to maximum
  analogWrite(enB, 255); // Set motor speed to maximum
  Serial.println("Moving right");
}

void moveLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255); // Set motor speed to maximum
  analogWrite(enB, 255); // Set motor speed to maximum
  Serial.println("Moving left");
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0); // Disable motor 1
  analogWrite(enB, 0); // Disable motor 2
  Serial.println("Motors stopped");
}
