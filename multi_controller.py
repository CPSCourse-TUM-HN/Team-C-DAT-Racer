import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import math


class TwoPlayerHandGestureController:
    def __init__(self, serial_port='/dev/cu.usbmodem1101', baudrate=9600):
        """
        Initialize the hand gesture controller for two players.
        Left side of screen controls Car 1, right side controls Car 2.

        Args:
            serial_port: Serial port for Arduino communication
            baudrate: Baud rate for serial communication
        """
        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,  # Two players
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # Initialize serial communication
        try:
            self.serial_conn = serial.Serial(serial_port, baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"Connected to Arduino on {serial_port}")
        except:
            self.serial_conn = None
            print("Arduino not connected - running in demo mode")

        # Control parameters for each car
        self.car1_speed = 0
        self.car1_angle = 0
        self.car2_speed = 0
        self.car2_angle = 0

        # Track which side has a hand
        self.left_hand_detected = False
        self.right_hand_detected = False

    def calculate_hand_openness(self, hand_landmarks):
        """
        Calculate how open the hand is (0 = fully closed fist, 1 = fully open).
        Based on the distance between fingertips and palm.
        """
        # Get wrist position (base of palm)
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

        # Get fingertip positions
        fingertips = [
            hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
        ]

        # Get MCP (metacarpophalangeal) joints for reference
        mcps = [
            hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP]
        ]

        # Calculate average distance ratio
        openness_ratios = []

        for i in range(1, 5):  # Skip thumb for more reliable detection
            # Distance from fingertip to wrist
            tip_to_wrist = math.sqrt(
                (fingertips[i].x - wrist.x) ** 2 +
                (fingertips[i].y - wrist.y) ** 2
            )

            # Distance from MCP to wrist (reference length)
            mcp_to_wrist = math.sqrt(
                (mcps[i - 1].x - wrist.x) ** 2 +
                (mcps[i - 1].y - wrist.y) ** 2
            )

            # Ratio (normalized by MCP distance)
            if mcp_to_wrist > 0:
                ratio = tip_to_wrist / mcp_to_wrist
                # Clamp between typical closed (0.8) and open (2.0) values
                normalized_ratio = (ratio - 0.8) / (2.0 - 0.8)
                normalized_ratio = max(0, min(1, normalized_ratio))
                openness_ratios.append(normalized_ratio)

        return np.mean(openness_ratios) if openness_ratios else 0

    def calculate_hand_tilt(self, hand_landmarks):
        """
        Calculate the horizontal tilt of the hand in degrees.
        Range: -45 to +45 degrees (negative = left tilt, positive = right tilt)
        """
        # Get index and pinky MCP for tilt calculation
        index_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        pinky_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP]

        # Calculate angle of the line from index to pinky
        dx = pinky_mcp.x - index_mcp.x
        dy = pinky_mcp.y - index_mcp.y

        # Calculate angle in radians and convert to degrees
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)

        # Clamp to -45 to +45 range
        tilt_angle = max(-45, min(45, angle_deg))

        return tilt_angle

    def send_commands_to_arduino(self):
        """
        Send control commands for both cars to Arduino.
        Format: "C1S<speed>A<angle>C2S<speed>A<angle>\n"
        """
        if self.serial_conn:
            # Create command for both cars
            command = f"C1S{int(self.car1_speed)}A{int(self.car1_angle)}C2S{int(self.car2_speed)}A{int(self.car2_angle)}\n"
            try:
                self.serial_conn.write(command.encode())
                print(f"Sent: {command.strip()}")
            except Exception as e:
                print(f"Failed to send command: {e}")
        else:
            print(
                f"Demo: Car1[S:{int(self.car1_speed)}% A:{int(self.car1_angle)}°] Car2[S:{int(self.car2_speed)}% A:{int(self.car2_angle)}°]")

    def draw_control_info(self, image, car1_speed, car1_angle, car2_speed, car2_angle):
        """
        Draw control information for both cars on the image.
        """
        h, w, _ = image.shape

        # Draw center dividing line
        cv2.line(image, (w // 2, 0), (w // 2, h), (255, 255, 255), 2)

        # Draw "PLAYER 1" and "PLAYER 2" labels
        cv2.putText(image, "PLAYER 1 (CAR 1)", (w // 4 - 60, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(image, "PLAYER 2 (CAR 2)", (3 * w // 4 - 60, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Car 1 info (left side)
        cv2.putText(image, f"Speed: {car1_speed:.0f}%", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        bar1_length = int(150 * (car1_speed / 100))
        cv2.rectangle(image, (10, 70), (160, 90), (100, 100, 100), 2)
        cv2.rectangle(image, (10, 70), (10 + bar1_length, 90), (0, 255, 0), -1)

        cv2.putText(image, f"Angle: {car1_angle:.0f}°", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Car 1 angle visualization
        center1_x = 85
        center1_y = 150
        radius = 30
        cv2.circle(image, (center1_x, center1_y), radius, (100, 100, 100), 2)
        angle1_rad = math.radians(-car1_angle)
        end1_x = int(center1_x + radius * math.cos(angle1_rad))
        end1_y = int(center1_y + radius * math.sin(angle1_rad))
        cv2.line(image, (center1_x, center1_y), (end1_x, end1_y), (0, 0, 255), 3)

        # Car 2 info (right side)
        car2_x_offset = w // 2 + 10
        cv2.putText(image, f"Speed: {car2_speed:.0f}%", (car2_x_offset, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        bar2_length = int(150 * (car2_speed / 100))
        cv2.rectangle(image, (car2_x_offset, 70), (car2_x_offset + 150, 90), (100, 100, 100), 2)
        cv2.rectangle(image, (car2_x_offset, 70), (car2_x_offset + bar2_length, 90), (0, 255, 0), -1)

        cv2.putText(image, f"Angle: {car2_angle:.0f}°", (car2_x_offset, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Car 2 angle visualization
        center2_x = car2_x_offset + 75
        center2_y = 150
        cv2.circle(image, (center2_x, center2_y), radius, (100, 100, 100), 2)
        angle2_rad = math.radians(-car2_angle)
        end2_x = int(center2_x + radius * math.cos(angle2_rad))
        end2_y = int(center2_y + radius * math.sin(angle2_rad))
        cv2.line(image, (center2_x, center2_y), (end2_x, end2_y), (0, 0, 255), 3)

        # Draw hand detection status
        left_status = "Hand Detected" if self.left_hand_detected else "No Hand"
        right_status = "Hand Detected" if self.right_hand_detected else "No Hand"
        left_color = (0, 255, 0) if self.left_hand_detected else (0, 0, 255)
        right_color = (0, 255, 0) if self.right_hand_detected else (0, 0, 255)

        cv2.putText(image, left_status, (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, left_color, 2)
        cv2.putText(image, right_status, (car2_x_offset, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, right_color, 2)

        # Draw connection status
        status = "Arduino Connected" if self.serial_conn else "Demo Mode"
        cv2.putText(image, status, (w // 2 - 60, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def run(self):
        """
        Main loop for two-player hand gesture control.
        """
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("\n" + "=" * 60)
        print("Two-Player Hand Gesture Car Controller Started")
        print("=" * 60)
        print("Controls:")
        print("  LEFT SIDE (Player 1/Car 1)  |  RIGHT SIDE (Player 2/Car 2)")
        print("  - Open hand = 0% speed      |  - Open hand = 0% speed")
        print("  - Closed fist = 100% speed  |  - Closed fist = 100% speed")
        print("  - Tilt hand = turn          |  - Tilt hand = turn")
        print("  - Press 'q' to quit")
        print("=" * 60 + "\n")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Flip the frame horizontally for selfie-view
            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape

            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb_frame.flags.writeable = False

            # Process the frame for hands
            results = self.hands.process(rgb_frame)

            # Convert back to BGR for OpenCV
            rgb_frame.flags.writeable = True
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

            # Reset detection flags
            self.left_hand_detected = False
            self.right_hand_detected = False

            # Temporary values for smoothing
            new_car1_speed = 0
            new_car1_angle = 0
            new_car2_speed = 0
            new_car2_angle = 0

            # Process detected hands
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Draw hand landmarks
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                        self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2)
                    )

                    # Get hand center position (using wrist)
                    wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                    hand_x = wrist.x * w

                    # Calculate control values
                    openness = self.calculate_hand_openness(hand_landmarks)
                    speed = openness * 100
                    tilt = self.calculate_hand_tilt(hand_landmarks)

                    # Determine which side the hand is on
                    if hand_x < w / 2:
                        # Left side - Car 1
                        self.left_hand_detected = True
                        new_car1_speed = speed
                        new_car1_angle = tilt
                    else:
                        # Right side - Car 2
                        self.right_hand_detected = True
                        new_car2_speed = speed
                        new_car2_angle = tilt

            # Smooth the values (simple low-pass filter)
            alpha = 0.7  # Smoothing factor

            # Update Car 1 controls
            if self.left_hand_detected:
                self.car1_speed = alpha * new_car1_speed + (1 - alpha) * self.car1_speed
                self.car1_angle = alpha * new_car1_angle + (1 - alpha) * self.car1_angle
            else:
                # No hand on left side - stop Car 1
                self.car1_speed = 0
                self.car1_angle = 0

            # Update Car 2 controls
            if self.right_hand_detected:
                self.car2_speed = alpha * new_car2_speed + (1 - alpha) * self.car2_speed
                self.car2_angle = alpha * new_car2_angle + (1 - alpha) * self.car2_angle
            else:
                # No hand on right side - stop Car 2
                self.car2_speed = 0
                self.car2_angle = 0

            # Send commands to Arduino for both cars
            self.send_commands_to_arduino()

            # Draw control information
            self.draw_control_info(frame, self.car1_speed, self.car1_angle,
                                   self.car2_speed, self.car2_angle)

            # Display the frame
            cv2.imshow('Two-Player Hand Gesture Car Controller', frame)

            # Break on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        if self.serial_conn:
            # Stop both cars
            self.car1_speed = 0
            self.car1_angle = 0
            self.car2_speed = 0
            self.car2_angle = 0
            self.send_commands_to_arduino()
            self.serial_conn.close()
            print("Arduino connection closed")


if __name__ == "__main__":
    # Create controller instance
    controller = TwoPlayerHandGestureController(serial_port='/dev/cu.usbmodem1101', baudrate=9600)

    # Run the controller
    controller.run()
