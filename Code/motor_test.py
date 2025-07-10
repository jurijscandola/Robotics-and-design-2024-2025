#!/usr/bin/env python3
"""
Motor Test Script for the ILM Module
This script allows testing of individual motors and movement patterns
for the 3-wheeled holonomic robot.

Features:
- Automated motor tests
- Direct serial communication mode
- Support for both integrated_movement.ino and basic_moveset.ino

Usage:
- Automated tests: python motor_test.py --test [test_name]
- Interactive mode: python motor_test.py --interactive
"""

import time
import argparse
import sys
import serial
from typing import Optional

# Import the Arduino communication class from existing module
try:
    from communication_test import ArduinoCommunicator, TagData
except ImportError:
    print("Error: apriltag_communication.py module not found.")
    print("Make sure this script is in the same directory as apriltag_communication.py")
    sys.exit(1)

# Default settings
# Updated to the correct port where Arduino is detected
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_TEST_DURATION = 2.0  # seconds
DEFAULT_SPEED = 50.0  # mm or arbitrary units
DEFAULT_MODE = "2WHEEL"  # Use 3-wheel mode by default
# Don't use acceleraton by default (for backward compatibility)
DEFAULT_ACCEL = "OFF"


class MotorTester:
    """Class to handle motor testing for the 3-wheeled robot"""

    def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE,
                 speed: float = DEFAULT_SPEED, mode: str = DEFAULT_MODE,
                 accel: str = DEFAULT_ACCEL):
        """Initialize the tester with connection parameters"""
        self.arduino = ArduinoCommunicator(port, baud_rate)
        self.connected = False
        self.speed = speed
        self.mode = mode
        self.accel = accel  # "ON" or "OFF"

    def connect(self) -> bool:
        """Establish connection to the Arduino"""
        self.connected = self.arduino.connect()
        if not self.connected:
            print("Failed to connect to Arduino.")
            return False

        # Optional: Send ping to verify connection
        ping_success = self.arduino.ping()
        if not ping_success:
            print(
                "Note: Arduino connected but not responding to ping. Continuing anyway.")
            # Don't set connected to False, just continue

        # Set the wheel mode and acceleration
        self._set_mode(self.mode)
        self._set_accel(self.accel)

        print("Successfully connected to Arduino.")
        return True

    def disconnect(self) -> None:
        """Disconnect from the Arduino"""
        if self.connected:
            self.arduino.disconnect()
            print("Disconnected from Arduino.")

    def _set_mode(self, mode: str) -> bool:
        """Set the wheel mode (2WHEEL or 3WHEEL)"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False

        # Ensure mode is capitalized
        mode = mode.upper()
        if mode not in ["2WHEEL", "3WHEEL"]:
            print(f"Invalid mode: {mode}. Must be 2WHEEL or 3WHEEL.")
            return False

        self.mode = mode
        command = f"MODE:{mode}\r\n"

        try:
            self.arduino.serial.write(command.encode())
            print(
                f"Set wheel mode to: {mode} {'(omnidirectional)' if mode == '3WHEEL' else '(differential drive)'}")
            time.sleep(0.1)  # Give Arduino time to process
            return True
        except Exception as e:
            print(f"Error setting wheel mode: {e}")
            return False

    def _set_accel(self, accel: str) -> bool:
        """Set the acceleration mode (ON or OFF)"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False

        # Ensure accel is capitalized
        accel = accel.upper()
        if accel not in ["ON", "OFF"]:
            print(f"Invalid acceleration mode: {accel}. Must be ON or OFF.")
            return False

        self.accel = accel
        command = f"ACCEL:{accel}\r\n"

        try:
            self.arduino.serial.write(command.encode())
            print(
                f"Set acceleration to: {accel} {'(smooth start/stop)' if accel == 'ON' else '(instant)'}")
            time.sleep(0.1)  # Give Arduino time to process
            return True
        except Exception as e:
            print(f"Error setting acceleration mode: {e}")
            return False

    def test_diagnostics(self) -> bool:
        """Run the built-in diagnostics command"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False

        print("\n=== Running Arduino Diagnostics ===")
        result = self.arduino.send_test_command()
        if not result:
            print("Diagnostics command failed.")
            return False        # Get additional response data
        response = self.arduino._get_response(1.0)
        print(f"Diagnostics Response: {response}")
        print("=== Diagnostics Complete ===")
        return True

    def move(self, direction: str, distance: float, duration: float) -> bool:
        """
        Send movement command in a specific direction

        Parameters:
        - direction: 'W'(forward), 'S'(backward), 'A'(Turn left), 'R'(Turn right), '4'(Rotate Right), '6'(Rotate Left)
                     '7'(Diagonal-forward_left), '9'(Diagonal-forward_right), '1'(Diagonal-backward_left), '3'(backward-backward_right)
        - distance: arbitrary units used by the Arduino for speed control
        - duration: how long to move in seconds
        """
        if not self.connected:
            print("Not connected to Arduino.")
            return False

        # Convert CW/CCW to actual directions the Arduino understands
        tag_direction = direction
        if direction == 'CW':
            tag_direction = 'R'  # Use right with specific tag_id for rotation
        elif direction == 'CCW':
            tag_direction = 'L'  # Use left with specific tag_id for rotation
        elif direction == 'FWL':
            tag_direction = 'Q'  # Use Q for diagonal forward-left
        elif direction == 'FWR':
            tag_direction = 'E'  # Use E for diagonal forward-right
        elif direction == 'BWL':
            tag_direction = 'Z'  # Use Z for diagonal backward-left
        elif direction == 'BWR':
            tag_direction = 'C'  # Use C for diagonal backward-right

        # Use tag_id=0 for regular movement, tag_id=99 for rotation
        tag_id = 99 if direction in ['CW', 'CCW'] else 0

        # Create tag data
        tag = TagData(tag_id=tag_id, distance=distance,
                      direction=tag_direction)

        print(
            f"Moving {direction} at speed {distance} for {duration} seconds...")
        success = self.arduino.send_tag_data(tag)

        if not success:
            print("Failed to send movement command.")
            return False

        # Wait for the specified duration
        time.sleep(duration)

        # Stop movement
        print("Stopping...")
        success = self.arduino.send_stop()
        if not success:
            print("Failed to send stop command.")
            return False

        return True

    def test_individual_motor(self, motor: str, duration: float = DEFAULT_TEST_DURATION, speed: float = DEFAULT_SPEED) -> bool:
        """
        Test an individual motor

        Parameters:
        - motor: 'left', 'right', or 'back'
        - duration: how long to run the test in seconds
        - speed: speed to run the motor at
        """
        if not self.connected:
            print("Not connected to Arduino.")
            return False

        print(f"\n=== Testing {motor.upper()} motor ===")

        # Using TAG commands with specific parameters to isolate motor movements
        # Based on the holonomic drive equations in moveRobot function:
        #   leftSpeed = -0.5 * vx - 0.866 * vy + omega
        #   rightSpeed = -0.5 * vx + 0.866 * vy + omega
        #   backSpeed = vx + omega

        success = False

        if motor.lower() == 'left':
            # To isolate left motor: vx=0, vy=-1, omega=0 (backward)
            print("Testing LEFT motor FORWARD...")
            tag = TagData(tag_id=1, distance=speed, direction='B')
            success = self.arduino.send_tag_data(tag)
        elif motor.lower() == 'right':
            # To isolate right motor: vx=0, vy=1, omega=0 (forward)
            print("Testing RIGHT motor FORWARD...")
            tag = TagData(tag_id=2, distance=speed, direction='F')
            success = self.arduino.send_tag_data(tag)
        elif motor.lower() == 'back':
            # To isolate back motor: vx=1, vy=0, omega=0 (right)
            print("Testing BACK motor FORWARD...")
            tag = TagData(tag_id=3, distance=speed, direction='R')
            success = self.arduino.send_tag_data(tag)
        else:
            print(f"Unknown motor: {motor}")
            return False

        if not success:
            print("Failed to send motor test command.")
            return False

        # Wait for the specified duration
        time.sleep(duration)

        # Stop the motor
        print(f"Stopping {motor} motor...")
        success = self.arduino.send_stop()
        if not success:
            print("Failed to send stop command.")
            return False

        print(f"=== {motor.upper()} motor test completed ===")
        return True

    def run_all_tests(self, duration: float = DEFAULT_TEST_DURATION, speed: float = DEFAULT_SPEED) -> bool:
        """Run a sequence of tests on all motors and movement patterns"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False

        tests = [
            # Individual motors
            ("left", "LEFT motor", self.test_individual_motor,
             ["left", duration, speed]),
            ("right", "RIGHT motor", self.test_individual_motor,
             ["right", duration, speed]),
            ("back", "BACK motor", self.test_individual_motor,
             ["back", duration, speed]),

            # Basic movements
            ("forward", "FORWARD movement", self.move, ["F", speed, duration]),
            ("backward", "BACKWARD movement",
             self.move, ["B", speed, duration]),
            ("left", "LEFT movement", self.move, ["L", speed, duration]),
            ("right", "RIGHT movement", self.move, ["R", speed, duration]),

            # Rotations
            ("clockwise", "CLOCKWISE rotation",
             self.move, ["CW", speed, duration]),
            ("counterclockwise", "COUNTERCLOCKWISE rotation",
             self.move, ["CCW", speed, duration]),
        ]

        print("\n=== Starting Comprehensive Motor Tests ===")

        # First run diagnostics
        self.test_diagnostics()
        time.sleep(1)

        # Run each test with a pause between them
        for test_id, test_name, test_func, test_args in tests:
            print(f"\nStarting test: {test_name}")
            try:
                test_func(*test_args)
            except Exception as e:
                print(f"Error during {test_name}: {e}")

            # Pause between tests
            time.sleep(0.5)

        print("\n=== Motor Tests Completed ===")
        return True

    def interactive_mode(self) -> None:
        """Run an interactive serial control session"""
        if hasattr(self, 'arduino') and hasattr(self.arduino, 'serial') and self.arduino.serial:
            # We'll use the existing serial connection
            ser = self.arduino.serial
            print("Using existing serial connection...")
        else:
            try:
                ser = serial.Serial(DEFAULT_PORT, DEFAULT_BAUD_RATE, timeout=1)
                time.sleep(2)  # Give Arduino time to reset
                ser.reset_input_buffer()
            except serial.SerialException as e:
                print(f"Error opening serial port: {e}")
                return

        print("\n==== Interactive Serial Control Mode ====")
        print("Available commands:")
        print("  W - (forward)")
        print("  S - (backward)")
        print("  A - (Turn left)")
        print("  D - (Turn right)")
        print("  Q - (Rotate left)")
        print("  E - (Rotate right)")
        print("  5 - (Stop)")
        print("  4 - (Slide Left)")
        print("  6 - (Slide Right)")
        print("  7 - (Diagonal Forward-Left)")
        print("  9 - (Diagonal Forward-Right")
        print("  1 - (Diagonal Backward-Left")
        print("  2 - (Diagonal Backward-Right)")
        print("  T. Send 'TEST' command")
        print("  TAG. Send TAG command")
        print("  C. Custom command")
        print("  ACC. Set acceleration (ON/OFF)")
        print("  M. Set wheel mode (2WHEEL/3WHEEL)")
        print("  B. Toggle DEBUG mode")
        print("  Z. Quit")

        while True:
            try:
                choice = input("\nEnter command number: ")

                if choice == 'W':
                    print("Sending FORWARD ('W') command...")
                    ser.write(b'W\r\n')


                elif choice == 'S':
                    print("Sending BACKWARD ('S') command...")
                    ser.write(b'S\r\n')
                    
                elif choice == 'A':
                    print("Sending TURN_LEFT ('A') command...")
                    ser.write(b'A\r\n')
                    
                elif choice == 'D':
                    print("Sending TURN_RIGHT ('D') command...")
                    ser.write(b'D\r\n')
                    
                elif choice == 'Q':
                    print("Sending ROTATE_LEFT ('Q') command...")
                    ser.write(b'Q\r\n')
                    
                elif choice == 'E':
                    print("Sending ROTATE_RIGHT ('E') command...")
                    ser.write(b'E\r\n')
                    
                elif choice == '4':
                    print("Sending SLIDE_LEFT ('4') command...")
                    ser.write(b'4\r\n')
                    
                elif choice == '6':
                    print("Sending SLIDE_RIGHT ('6') command...")
                    ser.write(b'6\r\n')
                    
                elif choice == '7':
                    print("Sending DIAGONAL_FORWARD_LEFT ('7') command...")
                    ser.write(b'7\r\n')
                    
                elif choice == '9':
                    print("Sending DIAGONAL_FORWARD_RIGHT ('9') command...")
                    ser.write(b'9\r\n')
                    
                elif choice == '1':
                    print("Sending DIAGONAL_BACKWARD_LEFT ('1') command...")
                    ser.write(b'1\r\n')
                    
                elif choice == '3':
                    print("Sending DIAGONAL_BACKWARD_RIGHT ('3') command...")
                    ser.write(b'3\r\n')
                    
                elif choice == '5':
                    print("Sending STOP ('5') command...")
                    ser.write(b'5\r\n')

                elif choice == 'T':
                    print("Sending TEST command...")
                    ser.write(b'TEST\r\n')

                elif choice == 'TAG':
                    tag_id = input("Enter tag ID (default: 0): ") or "0"
                    distance = input(
                        "Enter distance/speed (default: 100): ") or "100"
                    direction = input(
                        "Enter direction (W,A,S,D,Q,E): ").upper() or "W"

                    if direction not in ['W', 'A', 'S', 'D', 'Q', 'E']:
                        print("Invalid direction! Using W (forward).")
                        direction = 'W'

                    cmd = f"TAG:{tag_id},{distance},{direction}\r\n"
                    print(f"Sending TAG command: {cmd.strip()}")
                    ser.write(cmd.encode())

                elif choice == 'C':
                    custom_cmd = input("Enter custom command: ")
                    if custom_cmd:
                        if not custom_cmd.endswith('\r\n'):
                            custom_cmd += '\r\n'
                        print(f"Sending: {custom_cmd.strip()}")
                        ser.write(custom_cmd.encode())
                    else:
                        print("Empty command, not sending.")

                elif choice == 'ACC':
                    accel_mode = input("Set acceleration (ON/OFF): ").upper()
                    if accel_mode not in ['ON', 'OFF']:
                        print("Invalid acceleration mode! Using OFF.")
                        accel_mode = 'OFF'

                    cmd = f"ACCEL:{accel_mode}\r\n"
                    print(f"Setting acceleration to: {accel_mode}")
                    ser.write(cmd.encode())

                elif choice == 'M':
                    wheel_mode = input(
                        "Set wheel mode (2WHEEL/3WHEEL): ").upper()
                    if wheel_mode not in ['2WHEEL', '3WHEEL']:
                        print("Invalid wheel mode! Using 2WHEEL.")
                        wheel_mode = '2WHEEL'

                    cmd = f"MODE:{wheel_mode}\r\n"
                    print(f"Setting wheel mode to: {wheel_mode}")
                    ser.write(cmd.encode())
                
                elif choice == 'B':
                    debug_mode = input("Set debug mode (0=OFF/1=ON): ")
                    if debug_mode not in ['0', '1']:
                        print("Invalid debug mode! Using 0 (OFF).")
                        debug_mode = '0'
                    
                    cmd = f"DEBUG:{debug_mode}\r\n"
                    print(f"Setting debug mode to: {'ON' if debug_mode == '1' else 'OFF'}")
                    ser.write(cmd.encode())

                elif choice == 'Z':
                    print("Quitting...")
                    ser.write(b'5\r\n')  # Stop motors before exiting
                    break

                else:
                    print("Invalid choice!")
                    continue

                # Read and print the response
                print("\nArduino Response:")
                start_time = time.time()
                while (time.time() - start_time) < 1.0:
                    if ser.in_waiting:
                        try:
                            line = ser.readline().decode('utf-8').strip()
                            print(f"> {line}")
                        except UnicodeDecodeError:
                            print("> [Decode Error]")
                    else:
                        time.sleep(0.05)

            except KeyboardInterrupt:
                print("\nSending STOP command before exiting...")
                ser.write(b'S\r\n')
                break

            except Exception as e:
                print(f"Error: {e}")

        # Don't close the serial connection if it's from the Arduino object
        if not (hasattr(self, 'arduino') and hasattr(self.arduino, 'serial') and self.arduino.serial == ser):
            ser.close()
            print("Serial connection closed.")


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='Test motors on the ILM robot')
    parser.add_argument('--port', '-p', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--duration', '-d', type=float, default=DEFAULT_TEST_DURATION,
                        help=f'Duration for each test in seconds (default: {DEFAULT_TEST_DURATION})')
    parser.add_argument('--speed', '-s', type=float, default=DEFAULT_SPEED,
                        help=f'Motor speed (default: {DEFAULT_SPEED})')
    parser.add_argument('--test', '-t', choices=['all', 'left', 'right', 'back',
                                                 'forward', 'backward', 'rotate', 'diagnostics'],
                        help='Which automated test to run (default: all)')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Run in interactive mode for direct serial control')
    parser.add_argument('--accel', '-a', choices=['on', 'off'], default=DEFAULT_ACCEL.lower(),
                        help=f'Acceleration mode: on enables smooth start/stop, off uses instant speed changes (default: {DEFAULT_ACCEL.lower()})')
    parser.add_argument('--mode', '-m', choices=['2wheel', '3wheel'], default=DEFAULT_MODE.lower(),
                        help=f'Movement mode: 3wheel enables omnidirectional movement, 2wheel disables back wheel (default: {DEFAULT_MODE.lower()})')
    parser.add_argument('--debug', '-x', action='store_true',
                        help='Enable debug mode on the Arduino to see detailed output')
    return parser.parse_args()


def main():
    """Main function"""
    args = parse_args()

    # Convert acceleration and mode to uppercase
    accel = args.accel.upper()
    mode = args.mode.upper()
    tester = MotorTester(args.port, args.baud, args.speed, mode, accel)

    if args.interactive:
        # For interactive mode, we'll initialize the connection inside the method
        tester.interactive_mode()
        return 0

    # For standard operation, we need to connect first
    if not tester.connect():
        return 1

    # If debug mode is requested, enable it at the beginning
    if args.debug and hasattr(tester, 'arduino'):
        tester.arduino.set_debug_mode(True)
        print("Debug mode enabled on Arduino")

    try:
        if args.test == 'all' or args.test is None:
            tester.run_all_tests(args.duration, args.speed)
        elif args.test == 'left':
            tester.test_individual_motor('left', args.duration, args.speed)
        elif args.test == 'right':
            tester.test_individual_motor('right', args.duration, args.speed)
        elif args.test == 'back':
            tester.test_individual_motor('back', args.duration, args.speed)
        elif args.test == 'forward':
            tester.move('F', args.speed, args.duration)
        elif args.test == 'backward':
            tester.move('B', args.speed, args.duration)
        elif args.test == 'rotate':
            print("\n=== Testing rotation ===")
            tester.move('CW', args.speed, args.duration)
            time.sleep(0.5)
            tester.move('CCW', args.speed, args.duration)
        elif args.test == 'diagnostics':
            tester.test_diagnostics()
    finally:
        # Only stop and disconnect if we're in standard mode
        if not args.interactive and hasattr(tester, 'arduino'):
            tester.arduino.send_stop()
            tester.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
