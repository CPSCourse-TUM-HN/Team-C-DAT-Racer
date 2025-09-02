// Arduino Nano Car Code - Mounted on 4WD Car
// Receives commands via nRF24L01 and controls motors through L298N

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// nRF24L01 Configuration
RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001"; // Must match transmitter address

// L298N Motor Driver Pins
// Motor A (Left side motors)
const int ENA = 5;   // PWM pin for left motors speed
const int IN1 = 6;   // Left motors direction pin 1
const int IN2 = 7;   // Left motors direction pin 2

// Motor B (Right side motors)
const int ENB = 3;   // PWM pin for right motors speed
const int IN3 = 4;   // Right motors direction pin 1
const int IN4 = 8;   // Right motors direction pin 2

// Note: With 4WD and one L298N, we typically connect:
// - Front-left and Rear-left motors in parallel to Motor A
// - Front-right and Rear-right motors in parallel to Motor B

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

// Battery monitoring (optional)
const int BATTERY_PIN = A1; // Voltage divider to monitor battery
float batteryVoltage = 0;
unsigned long lastBatteryCheck = 0;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  Serial.println("Arduino Nano Car Starting...");
  
  // Initialize nRF24L01
  if (!radio.begin()) {
    Serial.println("ERROR: nRF24L01 not responding!");
    Serial.println("Check wiring: CE->9, CSN->10, SCK->13, MOSI->11, MISO->12");
    while (1) {
      // Blink LED to indicate error
      pinMode(LED_PIN, OUTPUT);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH); // Match transmitter power level
  radio.setDataRate(RF24_250KBPS); // Match transmitter data rate
  radio.startListening(); // This is the receiver
  
  Serial.println("nRF24L01 Initialized - Receiver Mode");
  
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
  
  // Check battery voltage
  // checkBattery();
  
  // Blink LED to indicate ready
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  Serial.println("Car Ready - Waiting for commands...");
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
    Serial.print("Received - Speed: ");
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
    Serial.println("Signal lost - Emergency stop!");
  }
  
  // Check battery voltage periodically (every 5 seconds)
  // if (millis() - lastBatteryCheck > 5000) {
  //   checkBattery();
  //   lastBatteryCheck = millis();
  // }
}

void controlCar(int speed, int angle) {
  // For 4 AA batteries (6V nominal, ~5V actual under load)
  // L298N has ~2V voltage drop, so motors get ~3-4V
  // We'll use higher PWM values to compensate
  
  // Convert speed percentage to PWM value
  // Using higher minimum PWM to overcome motor inertia with low voltage
  int baseSpeed;
  if (speed == 0) {
    baseSpeed = 0;
  } else {
    // Map 1-100% to 120-255 PWM (adjust if motors don't start)
    baseSpeed = map(speed, 1, 100, 120, 255);
  }
  
  // Calculate differential speed for turning
  float turnRatio = abs(angle) / 45.0; // 0 to 1
  
  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;
  
  if (angle < 0) {
    // Turn left - reduce left wheel speed
    leftSpeed = baseSpeed * (1 - turnRatio * 0.6); // Reduce up to 60%
    // For sharp turns, consider reversing inner wheel
    if (turnRatio > 0.8 && baseSpeed > 0) {
      // Sharp left turn - reverse left wheels
      leftSpeed = -baseSpeed * 0.3; // Reverse at 30% speed
    }
  } else if (angle > 0) {
    // Turn right - reduce right wheel speed  
    rightSpeed = baseSpeed * (1 - turnRatio * 0.6); // Reduce up to 60%
    // For sharp turns, consider reversing inner wheel
    if (turnRatio > 0.8 && baseSpeed > 0) {
      // Sharp right turn - reverse right wheels
      rightSpeed = -baseSpeed * 0.3; // Reverse at 30% speed
    }
  }
  
  // Apply to motors
  if (baseSpeed == 0) {
    stopCar();
  } else {
    // Left motors
    if (leftSpeed >= 0) {
      // Forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, abs(leftSpeed));
    } else {
      // Reverse (for sharp turns)
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, abs(leftSpeed));
    }
    
    // Right motors
    if (rightSpeed >= 0) {
      // Forward
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, abs(rightSpeed));
    } else {
      // Reverse (for sharp turns)
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, abs(rightSpeed));
    }
  }
}

void stopCar() {
  // Stop all motors - brake mode
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  currentSpeed = 0;
}

void checkBattery() {
  // Read battery voltage (if voltage divider is connected)
  // Use 10K + 10K resistor divider: Battery+ -> 10K -> A1 -> 10K -> GND
  int reading = analogRead(BATTERY_PIN);
  
  // Convert to voltage (assuming 5V reference and 2:1 divider)
  batteryVoltage = (reading / 1023.0) * 5.0 * 2.0;
  
  // 4 AA batteries: ~6V full, ~4V empty
  if (batteryVoltage > 0.5 && batteryVoltage < 4.5) {
    Serial.print("WARNING: Low battery! Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println("V");
    
    // Flash LED as warning
    for(int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
  } else if (batteryVoltage > 0.5) {
    Serial.print("Battery OK: ");
    Serial.print(batteryVoltage);
    Serial.println("V");
  }
}