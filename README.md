# DAT-Racer 🏎️
**vroom vroom** - Hand Gesture Controlled 4WD Arduino Car (Single & Multiplayer)

## Overview
DAT-Racer is a hand gesture-controlled 4WD car system that uses computer vision to track hand movements and wirelessly control Arduino-based cars. The system uses MediaPipe for hand tracking, OpenCV for computer vision processing, and nRF24L01 modules for wireless communication. 

**Now with multiplayer support!** Race against a friend with two cars controlled simultaneously.

## Game Modes

### 🎮 Single Player Mode
One player controls one car using hand gestures captured by the webcam.

### 🏁 Two Player Mode (Racing)
Two players control two separate cars simultaneously using split-screen hand tracking - left side controls Car 1, right side controls Car 2.

## System Architecture

### Single Player Setup
```
[Computer + Webcam] → [Arduino Uno + nRF24L01] → [Arduino Nano + nRF24L01 + 4WD Car]
     controller.py        Uno_code_final.ino         Nano_code_final.ino
```

### Two Player Setup
```
[Computer + Webcam] → [Arduino Uno + nRF24L01] → [Car 1: Arduino Nano + nRF24L01]
  controller.py    Uno_code_final_multi.ino    ↓
                                                    [Car 2: Arduino Nano + nRF24L01]
                                                    
Car 1: Nano_code_final_multi.ino (Address: "00001")
Car 2: Nano_code_final_multi.ino (Address: "00002" - modify in code)
```

## Features

### Hand Gesture Controls
- **Speed Control**: Hand openness controls speed
  - Open hand (spread fingers) = 0% speed (stop)
  - Closed fist = 100% speed (full speed)
  - Partial closure = proportional speed
- **Steering Control**: Hand tilt controls direction
  - Tilt left = turn left (-45° max)
  - Tilt right = turn right (+45° max)
  - Flat hand = straight driving

### Multiplayer Features (Two Player Mode)
- **Split Screen Control**: Screen divided into two zones
  - Left half controls Car 1 (Player 1)
  - Right half controls Car 2 (Player 2)
- **Independent Control**: Each car responds only to its designated hand
- **Visual Indicators**: Real-time display of both players' controls
- **Simultaneous Racing**: Both cars can be controlled at the same time

### Safety Features
- **Timeout Protection**: Cars stop if no commands received within 500ms
- **Signal Loss Detection**: Emergency stop if wireless connection lost
- **Hand Detection**: Cars stop when no hand is detected in their zone
- **Value Validation**: All speed and angle values are constrained to safe ranges
- **Independent Safety**: Each car has its own safety systems

### Visual Feedback
- Real-time hand landmark visualization
- Speed bar display (0-100%) for each player
- Angle indicator with visual compass for each car
- Connection status display
- Player zone indicators (Player 1/Player 2)
- Control value smoothing for stable operation

## Hardware Requirements

### Computer Side
- Computer with webcam
- Arduino Uno
- nRF24L01 wireless module
- Connecting wires
- USB cable for Arduino

### Car Side (Per Car)
- Arduino Nano
- nRF24L01 wireless module
- L298N motor driver
- 4WD car chassis with motors
- 4 AA battery pack (6V)
- LED indicator
- Connecting wires

**For Two Player Mode**: You need 2 complete car setups

## Wiring Diagrams

### Arduino Uno (Server) + nRF24L01
```
nRF24L01    Arduino Uno
VCC      →  3.3V
GND      →  GND
CE       →  Pin 9
CSN      →  Pin 10
SCK      →  Pin 13
MOSI     →  Pin 11
MISO     →  Pin 12
LED      →  Pin 13
```

### Arduino Nano (Each Car) + L298N + nRF24L01
```
nRF24L01    Arduino Nano
VCC      →  3.3V
GND      →  GND
CE       →  Pin 9
CSN      →  Pin 10
SCK      →  Pin 13
MOSI     →  Pin 11
MISO     →  Pin 12

L298N       Arduino Nano
ENA      →  Pin 5 (PWM)
IN1      →  Pin 6
IN2      →  Pin 7
ENB      →  Pin 3 (PWM)
IN3      →  Pin 4
IN4      →  Pin 8

LED      →  Pin A0
Battery  →  Pin A1 (optional voltage monitoring)
```

### Motor Connections
- **Motor A (L298N)**: Left side motors (front-left + rear-left in parallel)
- **Motor B (L298N)**: Right side motors (front-right + rear-right in parallel)

## Software Dependencies

### Python Requirements
```bash
pip install opencv-python
pip install mediapipe
pip install numpy
pip install pyserial
```

### Arduino Libraries
- SPI (built-in)
- nRF24L01 - Install "RF24" library by TMRh20
- RF24 (part of nRF24L01 library)

## Installation & Setup

### Single Player Setup

#### 1. Hardware Assembly
1. Wire the Arduino Uno with nRF24L01 module
2. Assemble the 4WD car with Arduino Nano, L298N, and nRF24L01
3. Connect motors to L298N driver
4. Install batteries and test power

#### 2. Arduino Programming
1. Upload `Uno_code_final.ino` to the Arduino Uno
2. Upload `Nano_code_final.ino` to the Arduino Nano
3. Verify wireless connection established

#### 3. Python Setup
1. Install required Python packages
2. Update serial port in `controller.py` (line 235):
   ```python
   controller = HandGestureController(serial_port='/dev/cu.usbmodem101', baudrate=9600)
   ```

### Two Player Setup

#### 1. Hardware Assembly
1. Wire the Arduino Uno with nRF24L01 module
2. Assemble TWO 4WD cars, each with Arduino Nano, L298N, and nRF24L01
3. Connect motors to L298N drivers on both cars
4. Install batteries in both cars

#### 2. Arduino Programming
1. Upload `Uno_code_final_multi.ino` to the Arduino Uno (server)
2. Upload `Nano_code_final_multi.ino` to Car 1's Arduino Nano
3. Modify `Nano_code_final_multi.ino` for Car 2:
   ```cpp
   // Change this line for Car 2:
   const byte address[6] = "00002"; // Car 2 address
   ```
4. Upload modified code to Car 2's Arduino Nano
5. Verify both cars receive initial stop commands

#### 3. Python Setup
1. Install required Python packages
2. Update serial port in `controller.py`:
   ```python
   controller = TwoPlayerHandGestureController(serial_port='/dev/cu.usbmodem1101', baudrate=9600)
   ```

## Usage

### Starting Single Player Mode
1. **Power on the car**
2. **Connect Arduino Uno** to computer via USB
3. **Run the controller**:
   ```bash
   python controller.py
   ```
4. **Position your hand** in front of the webcam
5. **Control the car** with hand gestures!

### Starting Two Player Mode
1. **Power on both cars**
2. **Connect Arduino Uno** to computer via USB
3. **Run the multiplayer controller**:
   ```bash
   python controller.py
   ```
4. **Player 1**: Position hand on LEFT side of screen
5. **Player 2**: Position hand on RIGHT side of screen
6. **Race!** Both players can control their cars simultaneously

### Controls
- **Stop**: Open your hand completely (spread all fingers)
- **Go**: Close your hand into a fist
- **Speed**: Partially close your hand for variable speed
- **Turn Left**: Tilt your hand to the left
- **Turn Right**: Tilt your hand to the right
- **Quit**: Press 'q' key in the video window

## Serial Monitor Commands & Debugging

### Single Player Mode

#### Arduino Uno Serial Output (9600 baud)
```
Received from Python: 'S75A-20'
Parsed: Speed=75% Angle=-20°
Sending to car... SUCCESS!
```

#### Arduino Nano Serial Output (9600 baud)
```
Received - Speed: 75% Angle: -20°
```

### Two Player Mode

#### Arduino Uno Serial Output (9600 baud)
```
Received: 'C1S50A-20C2S75A10'
Car 1: Speed=50% Angle=-20° [Car1:OK]
Car 2: Speed=75% Angle=10° [Car2:OK]
```

#### Car 1 Nano Serial Output
```
=====================================
Arduino Nano CAR 1 Starting...
Address: 00001 (Player 1 - Left Side)
=====================================
Car 1 Received - Speed: 50% Angle: -20°
```

#### Car 2 Nano Serial Output
```
=====================================
Arduino Nano CAR 2 Starting...
Address: 00002 (Player 2 - Right Side)
=====================================
Car 2 Received - Speed: 75% Angle: 10°
```

### Manual Testing Commands

#### Single Player (via Uno Serial Monitor)
```
S0A0     → Stop car, center steering
S50A0    → 50% speed, straight
S100A-45 → Full speed, maximum left turn
```

#### Two Player (via Uno Serial Monitor)
```
C1S0A0C2S0A0       → Stop both cars
C1S50A0C2S50A0     → Both cars 50% speed, straight
C1S100A-45C2S100A45 → Car 1 full left, Car 2 full right
```

## Troubleshooting

### Single Car Issues
- **No Arduino Connection**: Check USB cable and serial port in code
- **Poor Hand Detection**: Ensure good lighting and clear background
- **Car Not Responding**: Check nRF24L01 wiring (3.3V!), battery voltage
- **Weak Motors**: Check battery voltage (>5V), adjust PWM values in code

### Multiplayer Specific Issues

#### Car 1 works but Car 2 doesn't
- Verify Car 2's address is set to "00002" in its Nano code
- Check Car 2's nRF24L01 wiring and power
- Monitor Arduino Uno serial output for "[Car2:FAIL]" messages

#### Hands detected on wrong side
- Ensure players are positioned correctly (Player 1 = left, Player 2 = right)
- Check camera is not mirrored incorrectly
- Verify split-screen detection in Python output

#### Cross-control (hands controlling wrong car)
- Check address configuration in both Nano codes
- Verify Uno_code_final_multi.ino is uploaded (not single player version)
- Ensure controller.py is running (not controller.py)

#### Both cars move together
- This indicates both Nanos have the same address
- Re-upload code to Car 2 with address "00002"

## Customization

### Adjusting Sensitivity
In `controller.py`:
```python
# Hand detection confidence
min_detection_confidence=0.7    # Lower = more sensitive
min_tracking_confidence=0.5     # Lower = more sensitive

# Control smoothing
alpha = 0.7  # Higher = less smooth, more responsive
```

### Motor Performance Tuning
In `Nano_code_final.ino` or `Nano_code_final_multi.ino`:
```cpp
// Minimum PWM for motor startup
baseSpeed = map(speed, 1, 100, 120, 255);  // Increase 120 if motors don't start

// Turn sensitivity
leftSpeed = baseSpeed * (1 - turnRatio * 0.6);  // Increase 0.6 for sharper turns
```

### Wireless Range
Increase transmission power for longer range:
```cpp
radio.setPALevel(RF24_PA_HIGH);  // Options: LOW, HIGH, MAX
```

### Adding More Cars
To add a third car:
1. Define new address in Uno code: `const byte address3[6] = "00003";`
2. Add parsing for C3 commands in Uno code
3. Modify Python controller to handle 3+ hand zones
4. Upload modified Nano code with address "00003" to Car 3

## Competition Ideas

### Racing Modes
- **Time Trial**: Fastest lap around a track
- **Obstacle Course**: Navigate through cones
- **Capture the Flag**: Grab objects and return to base
- **Tag**: One car chases the other
- **Synchronized Dancing**: Both cars perform same movements

### Scoring System Ideas
- Add lap counters using IR sensors
- Implement checkpoint system
- Create penalty zones
- Add power-up areas with different speed limits

## Future Enhancements
- [ ] Support for 4+ players
- [ ] Mobile app control interface
- [ ] FPV camera integration on each car
- [ ] Obstacle detection with ultrasonic sensors
- [ ] Voice commands integration
- [ ] Online multiplayer over network
- [ ] AI opponent mode
- [ ] Gesture-based special moves (jump, spin, boost)
- [ ] LED strips for car customization
- [ ] Sound effects and music
- [ ] Replay system for races

## License
This project is open source. Feel free to modify and improve!

## Contributing
Pull requests are welcome! Please feel free to submit bug reports, feature requests, or improvements.

---

**Happy Racing! 🏁**

*May the best hand win!*
