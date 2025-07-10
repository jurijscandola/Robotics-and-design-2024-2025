#!/usr/bin/env python3
"""
UART Communication Interface for Raspberry Pi 4 and Arduino Mega
This module provides reliable serial communication between the Raspberry Pi
and Arduino using the UART protocol with error handling and retry mechanisms.
"""

import serial
import time
import logging
from dataclasses import dataclass
from typing import Optional, Dict, Any

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

# UART Communication constants
BAUD_RATE = 115200
DEFAULT_UART_PORT = '/dev/ttyACM0'  # Default port for Raspberry Pi 4
MSG_START = '<'
MSG_END = '>'
CMD_MOVE = 'MOV'
CMD_SPEED = 'SPEED'
CMD_STOP = 'STOP'
CMD_PING = 'PING'
CMD_SENS = 'SENS'
CMD_TAG = 'TAG'   # Command for AprilTag tracking with separate distance and speed
CMD_I2CM = 'I2CM'  # Command for I2C master communication

# Direction codes - aligned with integrated_movement.ino
DIR_STOP = 0              # Stop all movement
DIR_FORWARD = 1           # Move forward
DIR_BACKWARD = 2          # Move backward
DIR_TURN_LEFT = 3         # Arc turn left (forward while turning)
DIR_TURN_RIGHT = 4        # Arc turn right (forward while turning)
DIR_ROTATE_LEFT = 5       # Rotate left in place
DIR_ROTATE_RIGHT = 6      # Rotate right in place
DIR_SLIDE_LEFT = 7        # Left lateral movement
DIR_SLIDE_RIGHT = 8       # Right lateral movement
DIR_DIAGONAL_FORWARD_LEFT = 9    # Diagonal movement forward-left
DIR_DIAGONAL_FORWARD_RIGHT = 10  # Diagonal movement forward-right
DIR_DIAGONAL_BACKWARD_LEFT = 11  # Diagonal movement backward-left
DIR_DIAGONAL_BACKWARD_RIGHT = 12  # Diagonal movement backward-right


def get_direction_name(direction_code: int) -> str:
    """Convert direction code to readable string"""
    return {
        DIR_STOP: "STOP",
        DIR_FORWARD: "FORWARD",
        DIR_BACKWARD: "BACKWARD",
        DIR_TURN_LEFT: "TURN LEFT",
        DIR_TURN_RIGHT: "TURN RIGHT",
        DIR_ROTATE_LEFT: "ROTATE LEFT",
        DIR_ROTATE_RIGHT: "ROTATE RIGHT",
        DIR_SLIDE_LEFT: "SLIDE LEFT",
        DIR_SLIDE_RIGHT: "SLIDE RIGHT",
        DIR_DIAGONAL_FORWARD_LEFT: "DIAGONAL FORWARD-LEFT",
        DIR_DIAGONAL_FORWARD_RIGHT: "DIAGONAL FORWARD-RIGHT",
        DIR_DIAGONAL_BACKWARD_LEFT: "DIAGONAL BACKWARD-LEFT",
        DIR_DIAGONAL_BACKWARD_RIGHT: "DIAGONAL BACKWARD-RIGHT"
    }.get(direction_code, "UNKNOWN")


@dataclass
class TagData:
    """Container for movement data from tag tracking"""
    direction: int                # Direction to move (see direction constants)
    speed: int                    # Desired speed set by Python based on navigation goals
    tag_id: int                   # ID of the AprilTag being tracked
    # Physical distance to tag in cm (sent to Arduino for information)
    distance: Optional[float] = None
    timestamp: Optional[float] = None  # When this command was generated

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()

    def __str__(self) -> str:
        """String representation for debugging"""
        dist_str = f", Distance={self.distance:.1f}cm" if self.distance is not None else ""
        return f"Movement: Dir={get_direction_name(self.direction)}, DesiredSpeed={self.speed}{dist_str}"


class UARTCommunicator:
    def __init__(self, port: str = None, baud_rate: int = BAUD_RATE, debug: bool = False):
        """Initialize UART communication"""
        self.port = port if port else DEFAULT_UART_PORT
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.debug = debug
        self.min_speed = 30  # Default minimum speed
        self.max_speed = 255  # Default maximum speed

    def connect(self) -> bool:
        """Establish serial connection with Arduino"""
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.connected = True
            logger.info(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Close the serial connection"""
        if self.serial:
            self.serial.close()
        self.connected = False

    def _send_command(self, command: str) -> bool:
        """Send a command to Arduino and wait for acknowledgment"""
        if not self.connected:
            return False

        try:
            self.serial.write(command.encode('utf-8'))
            self.serial.flush()

            if self.debug:
                logger.debug(f"Sent: {command}")

            # Wait for acknowledgment
            response = self.serial.readline().decode().strip()
            if response.startswith('<ACK:'):
                return True
            else:
                logger.warning(f"Unexpected response: {response}")
                return False

        except Exception as e:
            logger.error(f"Communication error: {e}")
            return False

    def send_movement(self, direction: int, speed: int) -> bool:
        """Send movement command with direction and speed"""
        if not self.connected:
            return False

        # If direction is STOP, speed should be 0
        if direction == DIR_STOP:
            speed = 0

        # Constrain speed to valid range
        if speed > 0:
            speed = max(self.min_speed, min(self.max_speed, speed))

        command = f"{MSG_START}MOV:{direction},{speed}{MSG_END}"
        return self._send_command(command)

    def send_tag_data(self, tag_data: TagData) -> bool:
        """Send movement data from tag tracking, with separate distance and desired speed"""
        # If TAG command is needed, use it for proper distance+speed handling
        if tag_data.distance is not None:
            # Format: <TAG:id,distance,direction,desiredSpeed>
            # Arduino will apply obstacle avoidance scaling to the desired speed if needed
            command = f"{MSG_START}TAG:{tag_data.tag_id},{tag_data.distance:.1f},{tag_data.direction},{tag_data.speed}{MSG_END}"
            return self._send_command(command)
        else:
            # Fallback to regular movement command if no distance data
            return self.send_movement(tag_data.direction, tag_data.speed)

    def set_speed(self, max_speed: int, min_speed: int) -> bool:
        """Set motor speed parameters"""
        # Validate speed parameters
        max_speed = max(50, min(255, max_speed))
        # Ensure at least 5 difference
        min_speed = max(30, min(max_speed - 5, min_speed))

        # Store the values
        self.max_speed = max_speed
        self.min_speed = min_speed

        # Send to Arduino
        command = f"{MSG_START}SPEED:{max_speed},{min_speed}{MSG_END}"
        return self._send_command(command)

    def send_stop(self) -> bool:
        """Send stop command"""
        command = f"{MSG_START}STOP{MSG_END}"
        return self._send_command(command)

    def ping(self) -> bool:
        """Send ping command to check connection"""
        command = f"{MSG_START}PING{MSG_END}"
        return self._send_command(command)

    def set_wheel_mode(self, use_three_wheels: bool) -> bool:
        """Set wheel configuration mode

        Args:
            use_three_wheels: True for 3-wheel mode, False for 2-wheel mode

        Returns:
            bool: True if command was successful, False otherwise
        """
        mode = 1 if use_three_wheels else 0
        command = f"{MSG_START}MODE:{mode}{MSG_END}"
        return self._send_command(command)

    # Convenience methods for specific movement types

    def move_forward(self, speed: int) -> bool:
        """Move forward at the specified speed"""
        return self.send_movement(DIR_FORWARD, speed)

    def move_backward(self, speed: int) -> bool:
        """Move backward at the specified speed"""
        return self.send_movement(DIR_BACKWARD, speed)

    def turn_left(self, speed: int) -> bool:
        """Arc turn left (forward while turning left)"""
        return self.send_movement(DIR_TURN_LEFT, speed)

    def turn_right(self, speed: int) -> bool:
        """Arc turn right (forward while turning right)"""
        return self.send_movement(DIR_TURN_RIGHT, speed)

    def rotate_left(self, speed: int) -> bool:
        """Rotate left in place"""
        return self.send_movement(DIR_ROTATE_LEFT, speed)

    def rotate_right(self, speed: int) -> bool:
        """Rotate right in place"""
        return self.send_movement(DIR_ROTATE_RIGHT, speed)

    def slide_left(self, speed: int) -> bool:
        """Slide left (lateral movement)"""
        return self.send_movement(DIR_SLIDE_LEFT, speed)

    def slide_right(self, speed: int) -> bool:
        """Slide right (lateral movement)"""
        return self.send_movement(DIR_SLIDE_RIGHT, speed)

    def diagonal_forward_left(self, speed: int) -> bool:
        """Move diagonally forward and left"""
        return self.send_movement(DIR_DIAGONAL_FORWARD_LEFT, speed)

    def diagonal_forward_right(self, speed: int) -> bool:
        """Move diagonally forward and right"""
        return self.send_movement(DIR_DIAGONAL_FORWARD_RIGHT, speed)

    def diagonal_backward_left(self, speed: int) -> bool:
        """Move diagonally backward and left"""
        return self.send_movement(DIR_DIAGONAL_BACKWARD_LEFT, speed)

    def diagonal_backward_right(self, speed: int) -> bool:
        """Move diagonally backward and right"""
        return self.send_movement(DIR_DIAGONAL_BACKWARD_RIGHT, speed)

    def stop(self) -> bool:
        """Stop all movement"""
        return self.send_movement(DIR_STOP, 0)

    def send_i2c_command(self, command_code: int) -> bool:
        """Send an I2C command to the master controller

        Args:
            command_code: The I2C command code to send

        Returns:
            bool: True if command was successful, False otherwise
        """
        command = f"{MSG_START}{CMD_I2CM}:{command_code}{MSG_END}"
        return self._send_command(command)
