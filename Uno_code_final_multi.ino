// Arduino Uno Server Code - Controls Two Cars
// Receives commands from Python and forwards to both cars via nRF24L01

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// nRF24L01 Configuration
RF24 radio(9, 10); // CE, CSN pins

// Different addresses for each car
const byte address1[6] = "00001"; // Car 1 address
const byte address2[6] = "00002"; // Car 2 address

// Variables for command parsing
String inputString = "";
boolean stringComplete = false;

// Control values for each car
int car1Speed = 0;
int car1Angle = 0;
int car2Speed = 0;
int car2Angle = 0;

// LED indicators
const int LED_PIN = 13;
const int CAR1_LED = 12; // Optional: separate LED for Car 1 activity
const int CAR2_LED = 11; // Optional: separate LED for Car 2 activity

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
  Serial.println("Two-Car Controller Server");
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
  
  // Configure radio
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.stopListening(); // This is the transmitter
  
  // Setup LED indicators
  pinMode(LED_PIN, OUTPUT);
  pinMode(CAR1_LED, OUTPUT);
  pinMode(CAR2_LED, OUTPUT);
  
  // Print configuration
  Serial.println("nRF24L01 Configuration:");
  Serial.println(" - Mode: Transmitter");
  Serial.println(" - Power: HIGH");
  Serial.println(" - Data Rate: 250 KBPS");
  Serial.println(" - Car 1 Address: 00001");
  Serial.println(" - Car 2 Address: 00002");
  Serial.println("=================================");
  Serial.println("Server Ready for Two Cars!");
  Serial.println("Waiting for Python commands...");
  Serial.println("=================================");
  
  // Blink LEDs to indicate ready
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(CAR1_LED, HIGH);
    digitalWrite(CAR2_LED, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(CAR1_LED, LOW);
    digitalWrite(CAR2_LED, LOW);
    delay(200);
  }
  
  // Send initial stop commands to both cars
  Serial.println("Sending initial stop commands...");
  sendToCar1(0, 0);
  sendToCar2(0, 0);
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
  
  // Safety timeout - stop both cars if no data received for 1 second
  if (millis() - lastDataTime > TIMEOUT && (car1Speed > 0 || car2Speed > 0)) {
    Serial.println("Timeout - Stopping both cars");
    sendToCar1(0, 0);
    sendToCar2(0, 0);
    car1Speed = 0;
    car1Angle = 0;
    car2Speed = 0;
    car2Angle = 0;
    digitalWrite(LED_PIN, LOW);
    digitalWrite(CAR1_LED, LOW);
    digitalWrite(CAR2_LED, LOW);
  }
}

void processCommand() {
  // Debug: Show received command
  Serial.print("Received: '");
  Serial.print(inputString);
  Serial.println("'");
  
  // Parse command format: C1S<speed>A<angle>C2S<speed>A<angle>
  // Example: C1S50A-20C2S75A10
  
  int c1Index = inputString.indexOf("C1");
  int c2Index = inputString.indexOf("C2");
  
  if (c1Index != -1) {
    // Parse Car 1 command
    int c1SpeedIndex = inputString.indexOf('S', c1Index);
    int c1AngleIndex = inputString.indexOf('A', c1Index);
    
    if (c1SpeedIndex != -1 && c1AngleIndex != -1) {
      String speedStr = inputString.substring(c1SpeedIndex + 1, c1AngleIndex);
      car1Speed = speedStr.toInt();
      
      // Find end of Car 1 angle (either at C2 or end of string)
      int c1AngleEnd = (c2Index != -1) ? c2Index : inputString.length();
      String angleStr = inputString.substring(c1AngleIndex + 1, c1AngleEnd);
      car1Angle = angleStr.toInt();
      
      // Validate values
      car1Speed = constrain(car1Speed, 0, 100);
      car1Angle = constrain(car1Angle, -45, 45);
      
      Serial.print("Car 1: Speed=");
      Serial.print(car1Speed);
      Serial.print("% Angle=");
      Serial.print(car1Angle);
      Serial.print("° ");
      
      // Send to Car 1
      sendToCar1(car1Speed, car1Angle);
    }
  }
  
  if (c2Index != -1) {
    // Parse Car 2 command
    int c2SpeedIndex = inputString.indexOf('S', c2Index);
    int c2AngleIndex = inputString.indexOf('A', c2Index);
    
    if (c2SpeedIndex != -1 && c2AngleIndex != -1) {
      String speedStr = inputString.substring(c2SpeedIndex + 1, c2AngleIndex);
      car2Speed = speedStr.toInt();
      
      String angleStr = inputString.substring(c2AngleIndex + 1);
      car2Angle = angleStr.toInt();
      
      // Validate values
      car2Speed = constrain(car2Speed, 0, 100);
      car2Angle = constrain(car2Angle, -45, 45);
      
      Serial.print("Car 2: Speed=");
      Serial.print(car2Speed);
      Serial.print("% Angle=");
      Serial.print(car2Angle);
      Serial.print("°");
      
      // Send to Car 2
      sendToCar2(car2Speed, car2Angle);
    }
  }
  
  Serial.println();
  
  // Update LEDs
  digitalWrite(LED_PIN, (car1Speed > 0 || car2Speed > 0) ? HIGH : LOW);
  digitalWrite(CAR1_LED, car1Speed > 0 ? HIGH : LOW);
  digitalWrite(CAR2_LED, car2Speed > 0 ? HIGH : LOW);
  
  // Update last data time
  lastDataTime = millis();
}

void sendToCar1(int speed, int angle) {
  // Switch to Car 1 address
  radio.openWritingPipe(address1);
  
  // Prepare data
  dataToSend.speed = speed;
  dataToSend.angle = angle;
  
  // Send data
  bool result = radio.write(&dataToSend, sizeof(dataToSend));
  
  if (result) {
    Serial.print(" [Car1:OK]");
  } else {
    Serial.print(" [Car1:FAIL]");
  }
}

void sendToCar2(int speed, int angle) {
  // Switch to Car 2 address
  radio.openWritingPipe(address2);
  
  // Prepare data
  dataToSend.speed = speed;
  dataToSend.angle = angle;
  
  // Send data
  bool result = radio.write(&dataToSend, sizeof(dataToSend));
  
  if (result) {
    Serial.print(" [Car2:OK]");
  } else {
    Serial.print(" [Car2:FAIL]");
  }
}