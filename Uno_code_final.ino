// Arduino Uno Server Code - Connected to Computer
// Receives commands from Python and forwards to car via nRF24L01

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// nRF24L01 Configuration
RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001"; // Address for communication

// Variables for command parsing
String inputString = "";
boolean stringComplete = false;
int currentSpeed = 0;
int currentAngle = 0;

// LED indicator
const int LED_PIN = 13;
unsigned long lastDataTime = 0;
const unsigned long TIMEOUT = 1000; // 1 second timeout

// Data structure to send
struct DataPacket {
  int speed;
  int angle;
};

DataPacket dataToSend;

void setup() {
  // Initialize serial communication with computer (Python)
  Serial.begin(9600);
  Serial.println("=================================");
  Serial.println("Arduino Uno Server Starting...");
  Serial.println("=================================");
  
  // Initialize nRF24L01
  Serial.print("Initializing nRF24L01...");
  if (!radio.begin()) {
    Serial.println(" FAILED!");
    Serial.println("ERROR: nRF24L01 not responding!");
    Serial.println("Check wiring: CE->9, CSN->10, SCK->13, MOSI->11, MISO->12");
    Serial.println("Check: 3.3V power (NOT 5V!)");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  Serial.println(" SUCCESS!");
  
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH); // Start with LOW power (change to HIGH for longer range)
  radio.setDataRate(RF24_250KBPS); // Slower = more reliable
  radio.setRetries(5, 15); // 5 retries, 15*250us delay between retries
  radio.stopListening(); // This is the transmitter
  
  // Setup LED indicator
  pinMode(LED_PIN, OUTPUT);
  
  // Print configuration
  Serial.println("nRF24L01 Configuration:");
  Serial.println(" - Mode: Transmitter");
  Serial.println(" - Power: LOW (close range)");
  Serial.println(" - Data Rate: 250 KBPS");
  Serial.println(" - Address: 00001");
  Serial.println("=================================");
  Serial.println("Arduino Uno Server Ready!");
  Serial.println("Waiting for Python commands...");
  Serial.println("=================================");
  
  // Blink LED to indicate ready
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  // Send initial stop command to car
  Serial.println("Sending initial stop command...");
  dataToSend.speed = 0;
  dataToSend.angle = 0;
  bool result = radio.write(&dataToSend, sizeof(dataToSend));
  if (result) {
    Serial.println("Initial stop command sent successfully");
  } else {
    Serial.println("Warning: Could not send initial command (car may be off)");
  }
}

void loop() {
  // Check for incoming data from Python
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    // Check if we received a complete command (ends with newline)
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
  // Process complete command
  if (stringComplete) {
    processCommand();
    inputString = "";
    stringComplete = false;
  }
  
  // Safety timeout - stop car if no data received for 1 second
  if (millis() - lastDataTime > TIMEOUT && currentSpeed > 0) {
    currentSpeed = 0;
    currentAngle = 0;
    dataToSend.speed = 0;
    dataToSend.angle = 0;
    
    bool result = radio.write(&dataToSend, sizeof(dataToSend));
    if (!result) {
      Serial.println("WARNING: Failed to send stop command!");
    }
    
    digitalWrite(LED_PIN, LOW);
    Serial.println("Timeout - Car stopped");
  }
}

void processCommand() {
  // Debug: Show received command
  Serial.print("Received from Python: '");
  Serial.print(inputString);
  Serial.println("'");
  
  // Parse command format: S<speed>A<angle>
  int speedIndex = inputString.indexOf('S');
  int angleIndex = inputString.indexOf('A');
  
  if (speedIndex != -1 && angleIndex != -1) {
    // Extract speed value
    String speedStr = inputString.substring(speedIndex + 1, angleIndex);
    currentSpeed = speedStr.toInt();
    
    // Extract angle value
    String angleStr = inputString.substring(angleIndex + 1);
    currentAngle = angleStr.toInt();
    
    // Validate values
    currentSpeed = constrain(currentSpeed, 0, 100);
    currentAngle = constrain(currentAngle, -45, 45);
    
    Serial.print("Parsed: Speed=");
    Serial.print(currentSpeed);
    Serial.print("% Angle=");
    Serial.print(currentAngle);
    Serial.println("°");
    
    // Prepare data packet
    dataToSend.speed = currentSpeed;
    dataToSend.angle = currentAngle;
    
    // Send to car via nRF24L01
    Serial.print("Sending to car... ");
    bool result = radio.write(&dataToSend, sizeof(dataToSend));
    
    // Update LED (on when moving, off when stopped)
    digitalWrite(LED_PIN, currentSpeed > 0 ? HIGH : LOW);
    
    // Report status back to Python
    if (result) {
      Serial.println("SUCCESS!");
      Serial.print("  → Sent to car: Speed=");
      Serial.print(currentSpeed);
      Serial.print("% Angle=");
      Serial.print(currentAngle);
      Serial.println("°");
    } else {
      Serial.println("FAILED!");
      Serial.println("  ✗ Transmission failed - possible causes:");
    }
    
    // Update last data time
    lastDataTime = millis();
  } else {
    Serial.print("ERROR: Invalid command format. Expected S<speed>A<angle>, got: ");
    Serial.println(inputString);
  }
}