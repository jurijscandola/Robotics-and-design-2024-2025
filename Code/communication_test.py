"""
Simple AprilTag Communication Module
Handles serial communication with Arduino for robot movement control
"""

import serial
import time
import logging
from dataclasses import dataclass
from typing import Optional, Tuple

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Serial communication settings
# Updated to the correct port where Arduino is detected
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
RETRY_COUNT = 3
RETRY_DELAY = 0.1


@dataclass
class TagData:
    """Simple container for AprilTag detection data"""
    tag_id: int
    distance: float
    direction: str
    mode: str = None        # Optional wheel mode: "2WHEEL" or "3WHEEL"
    acceleration: str = None  # Optional acceleration: "ON" or "OFF"
    timestamp: float = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


class ArduinoCommunicator:
    """Handles serial communication with Arduino"""

    def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE):
        """Initialize the communicator with given port and baud rate"""
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.last_tag = None
        self.last_position = None

    def connect(self) -> bool:
        """Establish connection to Arduino"""
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            # Give Arduino more time to reset and initialize
            time.sleep(2)

            # Clear any initialization data that might be in the buffer
            if self.serial.in_waiting:
                initial_data = self.serial.read(self.serial.in_waiting)
                logger.debug(f"Cleared initialization data: {initial_data}")

            self.connected = True
            logger.info(f"Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False

    def disconnect(self) -> None:
        """Close the serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("Disconnected from Arduino")
        self.connected = False

    def _send_command(self, command: str) -> bool:
        """Send a command to Arduino with retry logic"""
        if not self.connected or not self.serial:
            logger.warning("Cannot send command: not connected")
            return False

        # Ensure proper line endings (carriage return + newline)
        # This matches the Arduino IDE Serial Monitor default behavior
        if not command.endswith('\r\n'):
            if command.endswith('\n'):
                command = command.rstrip('\n') + '\r\n'
            else:
                command = command + '\r\n'

        for attempt in range(RETRY_COUNT):
            try:
                self.serial.write(command.encode())
                self.serial.flush()
                return True
            except Exception as e:
                logger.warning(
                    f"Send failed (attempt {attempt+1}/{RETRY_COUNT}): {e}")
                time.sleep(RETRY_DELAY)

        logger.error("Failed to send command after retries")
        self.connected = False
        return False

    def _get_response(self, timeout=0.5) -> str:
        """Get response from Arduino with timeout"""
        if not self.connected or not self.serial:
            return ""

        start_time = time.time()
        response = ""
        response_received = False

        while (time.time() - start_time) < timeout:
            if self.serial.in_waiting > 0:
                response_received = True
                chunk = self.serial.readline().decode('ascii', errors='ignore').strip()
                if chunk:
                    response += chunk + " "
            elif response_received:
                # If we've already received data and buffer is now empty, we can exit
                break
            else:
                # Small delay to prevent CPU hogging while waiting for response
                time.sleep(0.01)

        return response.strip()

    def send_test_command(self) -> bool:
        """Send test command and verify Arduino response"""
        success = self._send_command("TEST\n")
        if not success:
            return False

        # Wait for response
        response = self._get_response(1.0)  # Longer timeout for test command
        logger.info(f"Arduino TEST response: {response}")

        # Accept any response as valid
        return len(response) > 0

    def ping(self) -> bool:
        """Ping Arduino to check if it's responsive"""
        if not self.connected:
            return False

        # Clear any pending data
        if self.serial and self.serial.in_waiting:
            self.serial.read(self.serial.in_waiting)

        success = self._send_command("PING\n")
        if not success:
            return False

        # Check for any response within timeout
        response = self._get_response(0.5)
        return len(response) > 0

    def send_tag_data(self, tag_data: TagData) -> bool:
        """Send tag detection data to Arduino"""
        if not self.connected:
            return False

        # Start with the basic format
        command = f"TAG:{tag_data.tag_id},{tag_data.distance},{tag_data.direction}"

        # Add mode parameter if present
        if tag_data.mode:
            command += f",MODE:{tag_data.mode}"

        # Add acceleration parameter if present
        if tag_data.acceleration:
            command += f",ACCEL:{tag_data.acceleration}"

        # Add newline terminator
        command += "\n"

        success = self._send_command(command)

        if success:
            self.last_tag = tag_data
            response = self._get_response(0.2)
            if response:
                logger.debug(f"Arduino response: {response}")

        return success

    def send_position(self, x: float, y: float, theta: float) -> bool:
        """Send position data to Arduino"""
        if not self.connected:
            return False

        # Format: POS:x,y,theta
        command = f"POS:{x:.1f},{y:.1f},{theta:.3f}\n"
        success = self._send_command(command)

        if success:
            self.last_position = (x, y, theta)

        return success

    def send_stop(self) -> bool:
        """Send stop command to Arduino"""
        return self._send_command("STOP\n")

    def send_clear(self) -> bool:
        """Send clear tag data command to Arduino"""
        return self._send_command("CLEAR\n")

    def set_speed(self, max_speed: int, min_speed: int) -> bool:
        """
        Set the maximum and minimum motor speeds on the Arduino

        Parameters:
        - max_speed: The maximum motor speed (50-255)
        - min_speed: The minimum motor speed (30-max_speed)

        Returns:
        - True if the command was sent successfully
        """
        # Ensure values are in valid ranges
        max_speed = max(50, min(255, max_speed))
        min_speed = max(30, min(max_speed, min_speed))

        # Format the command
        command = f"SPEED:{max_speed},{min_speed}\n"
        success = self._send_command(command)

        if success:
            logger.info(
                f"Speed parameters sent: MAX={max_speed}, MIN={min_speed}")

        return success

    def set_debug_mode(self, enabled: bool) -> bool:
        """
        Enable or disable debug mode on the Arduino

        Parameters:
        - enabled: True to enable debug mode, False to disable

        Returns:
        - True if the command was sent successfully
        """
        debug_value = 1 if enabled else 0
        command = f"DEBUG:{debug_value}\n"
        success = self._send_command(command)

        if success:
            logger.info(f"Debug mode {'enabled' if enabled else 'disabled'}")

        return success


def main():
    """Demo usage of the ArduinoCommunicator class"""
    communicator = ArduinoCommunicator()
    if communicator.connect():
        # Send test command to verify Arduino is responding
        print("Sending TEST command to Arduino...")
        test_success = communicator.send_test_command()
        print(
            f"Arduino TEST response: {'Successful' if test_success else 'Failed'}")

        # Send test commands
        tag = TagData(tag_id=1, distance=50.0, direction='W')
        communicator.send_tag_data(tag)
        time.sleep(1)
        communicator.send_position(100, 200, 1.5)
        time.sleep(1)
        communicator.send_stop()
        communicator.disconnect()
    else:
        logger.error("Demo failed: Could not connect to Arduino")


if __name__ == "__main__":
    main()
