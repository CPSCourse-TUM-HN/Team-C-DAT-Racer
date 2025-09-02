// Arduino Nano Car 1 Code - Player 1 (Left Side)
// This code goes on the FIRST car's Nano
// Receives commands via nRF24L01 on address "00001"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// nRF24L01 Configuration
RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001"; // Car 1 address - DIFFERENT FROM CAR 2!

// L298N Motor Driver Pins
const int ENA = 5;   // PWM pin for left motors speed
const int IN1 = 6;   // Left motors direction pin 1
const int IN2 = 7;   // Left motors direction pin 2
const int ENB = 3;   // PWM pin for right motors speed
const int IN3 = 4;   // Right motors direction pin 1
const int IN4 = 8;   // Right motors direction pin 2

// Data structure to receive
struct DataPacket {
  int speed;
  int angle;
};

DataPacket receivedData;

// Control variables
int currentSpeed = 0;
int currentAngle = 0;
unsigned long lastDataTime = 0;
const unsigned long TIMEOUT = 500; // Stop if no data for 500ms

// LED indicator
const int LED_PIN = A0; // Using analog pin as digital for LED

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  Serial.println("=====================================");
  Serial.println("Arduino Nano CAR 1 Starting...");
  Serial.println("Address: 00001 (Player 1 - Left Side)");
  Serial.println("=====================================");
  
  // Initialize nRF24L01
  if (!radio.begin()) {
    Serial.println("ERROR: nRF24L01 not responding!");
    while (1) {
      pinMode(LED_PIN, OUTPUT);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  
  Serial.println("nRF24L01 Initialized - Car 1 Receiver");
  
  // Setup motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  
  // Stop all motors initially
  stopCar();
  
  // Blink LED to indicate ready
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  Serial.println("Car 1 Ready - Waiting for commands...");
}

void loop() {
  // Check for wireless data
  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));
    
    // Update control values
    currentSpeed = receivedData.speed;
    currentAngle = receivedData.angle;
    
    // Validate values
    currentSpeed = constrain(currentSpeed, 0, 100);
    currentAngle = constrain(currentAngle, -45, 45);
    
    // Apply motor control
    controlCar(currentSpeed, currentAngle);
    
    // Update LED (blink pattern based on speed)
    if (currentSpeed > 0) {
      digitalWrite(LED_PIN, (millis() / 200) % 2); // Blink when moving
    } else {
      digitalWrite(LED_PIN, LOW); // Off when stopped
    }
    
    // Debug output
    Serial.print("Car 1 Received - Speed: ");
    Serial.print(currentSpeed);
    Serial.print("% Angle: ");
    Serial.print(currentAngle);
    Serial.println("°");
    
    lastDataTime = millis();
  }
  
  // Safety timeout - stop if no data received
  if (millis() - lastDataTime > TIMEOUT && currentSpeed > 0) {
    stopCar();
    digitalWrite(LED_PIN, LOW);
    Serial.println("Car 1: Signal lost - Emergency stop!");
  }
}

void controlCar(int speed, int angle) {
  // More precise speed mapping for full range control
  int baseSpeed;
  
  if (speed == 0) {
    baseSpeed = 0;
  } else if (speed <= 10) {
    // 1-10% = very slow (PWM 80-100)
    baseSpeed = map(speed, 1, 10, 80, 100);
  } else if (speed <= 30) {
    // 11-30% = slow to medium (PWM 100-150)
    baseSpeed = map(speed, 11, 30, 100, 150);
  } else if (speed <= 60) {
    // 31-60% = medium speed (PWM 150-200)
    baseSpeed = map(speed, 31, 60, 150, 200);
  } else {
    // 61-100% = fast to max (PWM 200-255)
    baseSpeed = map(speed, 61, 100, 200, 255);
  }
  
  // Calculate differential speed for turning
  float turnRatio = abs(angle) / 45.0; // 0 to 1
  
  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;
  
  if (angle < 0) {
    // Turn left - reduce left wheel speed
    leftSpeed = baseSpeed * (1 - turnRatio * 0.6);
    if (turnRatio > 0.8 && baseSpeed > 0) {
      leftSpeed = -baseSpeed * 0.3; // Sharp turn - reverse
    }
  } else if (angle > 0) {
    // Turn right - reduce right wheel speed  
    rightSpeed = baseSpeed * (1 - turnRatio * 0.6);
    if (turnRatio > 0.8 && baseSpeed > 0) {
      rightSpeed = -baseSpeed * 0.3; // Sharp turn - reverse
    }
  }
  
  // Apply to motors
  if (baseSpeed == 0) {
    stopCar();
  } else {
    // Left motors
    if (leftSpeed >= 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, abs(leftSpeed));
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, abs(leftSpeed));
    }
    
    // Right motors
    if (rightSpeed >= 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, abs(rightSpeed));
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, abs(rightSpeed));
    }
  }
  
  // Debug output for speed mapping
  Serial.print("Speed: ");
  Serial.print(speed);
  Serial.print("% -> Base PWM: ");
  Serial.print(baseSpeed);
  Serial.print(" | Final PWM - L: ");
  Serial.print(abs(leftSpeed));
  Serial.print(" R: ");
  Serial.println(abs(rightSpeed));
}

void stopCar() {
  // Stop all motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  currentSpeed = 0;
}