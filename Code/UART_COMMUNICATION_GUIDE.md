# UART Communication Guide for Robot Control

This guide explains how to use the UART communication module for communication between the Raspberry Pi 4 and Arduino Mega for robot control.

## Overview

The UART Communication Module provides a robust serial communication interface between the Raspberry Pi and Arduino with the following features:

- Message framing with start/end markers
- Reliable command acknowledgment
- Auto-detection of Arduino ports
- Support for numeric direction codes (0-12)
- Enhanced error handling
- Direct motor control capability
- Sensor data retrieval
- I2C integration with master controller

## Hardware Setup

1. Connect the Raspberry Pi to the Arduino Mega using a USB cable
2. Make sure the Arduino is powered on
3. The system will automatically detect the port

## Available Commands

The following commands are supported:

| Command | Description                 | Format                                     |
| ------- | --------------------------- | ------------------------------------------ |
| `PING`  | Check connection            | `<PING>`                                   |
| `STOP`  | Stop all motors             | `<STOP>`                                   |
| `MOVE`  | Move in specified direction | `<MOVE:direction,speed>`                   |
| `TAG`   | Send AprilTag data          | `<TAG:id,distance,direction>`              |
| `SENS`  | Request sensor data         | `<SENS>`                                   |
| `MCTL`  | Direct motor control        | `<MCTL:left_speed,right_speed,back_speed>` |
| `SPEED` | Set speed parameters        | `<SPEED:max_speed,min_speed>`              |
| `TEST`  | Run diagnostics             | `<TEST>`                                   |
| `I2CM`  | Send I2C command to master  | `<I2CM:command>`                           |

### Direction Codes

The following numeric direction codes are used:

- `0` = STOP
- `1` = FORWARD
- `2` = BACKWARD
- `3` = LEFT (Arc turn)
- `4` = RIGHT (Arc turn)
- `5` = ROTATE LEFT (in place)
- `6` = ROTATE RIGHT (in place)
- `7` = SLIDE LEFT (lateral movement)
- `8` = SLIDE RIGHT (lateral movement)
- `9` = DIAGONAL FORWARD-LEFT
- `10` = DIAGONAL FORWARD-RIGHT
- `11` = DIAGONAL BACKWARD-LEFT
- `12` = DIAGONAL BACKWARD-RIGHT

## Python Usage Examples

### Basic Connection

```python
from uart_communication import UARTCommunicator

# Connect to Arduino (auto-detection)
comm = UARTCommunicator()

# Or specify port manually
# comm = UARTCommunicator(port='/dev/ttyACM0')

# Check if connected
if comm.connected:
    print(f"Connected to Arduino on {comm.port}")
else:
    print("Failed to connect")
```

### Movement Commands

```python
# Basic movement commands
comm.send_movement(direction=1, speed=150)  # Move forward at speed 150
comm.send_movement(direction=2, speed=150)  # Move backward
comm.send_movement(direction=0)             # Stop

# Turning and Rotation
comm.send_movement(direction=3, speed=150)  # Arc turn left (forward while turning)
comm.send_movement(direction=4, speed=150)  # Arc turn right (forward while turning)
comm.send_movement(direction=5, speed=100)  # Rotate left in place
comm.send_movement(direction=6, speed=100)  # Rotate right in place

# Omnidirectional movement
comm.send_movement(direction=7, speed=120)  # Slide left (lateral movement)
comm.send_movement(direction=8, speed=120)  # Slide right (lateral movement)

# Diagonal movement
comm.send_movement(direction=9, speed=130)   # Diagonal forward-left
comm.send_movement(direction=10, speed=130)  # Diagonal forward-right
comm.send_movement(direction=11, speed=130)  # Diagonal backward-left
comm.send_movement(direction=12, speed=130)  # Diagonal backward-right
```

### Direct Motor Control

```python
# Control each motor directly
# Parameters: left_speed, right_speed, back_speed (-255 to 255)
# Positive values = forward, Negative values = backward

# Spin in place (left backward, right forward, back motor turn)
comm.set_motor_speeds(-150, 150, 150)

# Custom movement
comm.set_motor_speeds(100, 150, 0)
```

### Omnidirectional Movement Using Helper Methods

```python
# These helper methods make the code more readable
# and provide a cleaner interface for movement commands

# Basic movements
comm.move_forward(150)
comm.move_backward(150)
comm.stop()

# Turning and rotation
comm.turn_left(130)       # Arc turn (moves forward while turning)
comm.turn_right(130)      # Arc turn (moves forward while turning)
comm.rotate_left(80)      # Rotate in place
comm.rotate_right(80)     # Rotate in place

# Lateral movement
comm.slide_left(120)      # Pure lateral movement to the left
comm.slide_right(120)     # Pure lateral movement to the right

# Diagonal movement
comm.diagonal_forward_left(130)
comm.diagonal_forward_right(130)
comm.diagonal_backward_left(130)
comm.diagonal_backward_right(130)
```

### Tag Data Commands

```python
from uart_communication import TagData

# Create a tag data object
tag = TagData(tag_id=1, distance=150, direction=1)  # Forward

# Send it to Arduino
comm.send_tag_data(tag)
```

### Sensor Data

```python
# Request sensor data
sensor_data = comm.request_sensor_data()

if sensor_data:
    print(f"Front: {sensor_data.front}cm")
    print(f"Front Left: {sensor_data.front_left}cm")
    print(f"Front Right: {sensor_data.front_right}cm")
    print(f"Back: {sensor_data.back}cm")
    print(f"Back Left: {sensor_data.back_left}cm")
    print(f"Back Right: {sensor_data.back_right}cm")

    # Check minimum distance in front
    min_front = sensor_data.min_front
    print(f"Minimum front distance: {min_front}cm")
```

## Testing the Communication

Use the included test script to verify the communication:

```bash
# Run automatic test
python test_uart_communication.py

# Specify a port manually
python test_uart_communication.py -p /dev/ttyACM0

# Run in interactive mode
python test_uart_communication.py -i

# Show debug information
python test_uart_communication.py -d
```

## Troubleshooting

If you're having connection issues:

1. Check if the Arduino is properly connected and powered
2. Make sure the correct sketch is uploaded to the Arduino
3. Try unplugging and reconnecting the Arduino
4. Check the baud rate (default: 115200)
5. Check for errors in the Arduino serial monitor

## Arduino Setup

The Arduino needs to have the matching firmware with UART message framing support. Make sure you've loaded the `integrated_movement.ino` sketch onto your Arduino Mega.

## I2C Integration

The system supports I2C integration with the master controller using the following commands:

### I2C Configuration

- **Module Address**: `0x04` (Localization module's address)
- **Master Address**: `0x01` (Master controller's address)

### I2C Commands

- **ASK_READY_CMD (0b00011)**: Master checks if modules are ready
- **TELL_READY_CMD (0b11100)**: Module responds that it's ready
- **GO_IDLE_CMD (0b10111)**: Master tells module to enter idle mode
- **GO_ACTIVE_CMD (0b11000)**: Master tells module to enter active mode

### Using I2C Through UART

You can pass I2C commands through the UART interface using:

```python
# Send a command to the master through Arduino
comm.send_command(f"I2CM:{command_code}")

# Example: Tell module to enter active mode
comm.send_command("I2CM:24")  # 24 is decimal for 0b11000 (GO_ACTIVE_CMD)
```

## Advanced Usage

For more advanced usage and customization options, see the full documentation in the module's docstrings.
