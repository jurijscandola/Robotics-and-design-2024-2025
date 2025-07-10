# AprilTag UART Integration

This system integrates the AprilTag recognition system with UART communication for robust robot control. The implementation uses the `uart_communication.py` module, which provides more reliable communication features compared to the older `raspy_communication.py`.

## Key Features

- **Message framing** with start/end markers for reliable communication
- **Command acknowledgment** to ensure commands are received
- **Auto-detection** of Arduino serial ports
- **Standardized direction codes** (0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT, 5=ROTATE LEFT, 6=ROTATE RIGHT)
- **Enhanced error handling** with retries and timeouts
- **Direct motor control** capabilities
- **Sensor data retrieval** from ultrasonic sensors
- **I2C integration** with master controller for coordination

## Available Scripts

### Main Scripts

- **`apriltag_uart_controller.py`**: Main AprilTag detection and robot control script with comprehensive features
- **`uart_communication.py`**: The UART communication library
- **`communication_test.py`**: Test script for UART communication
- **`camera_apriltag_test.py`**: Test script for AprilTag detection without robot control
- **`monitor_sensor_data_fixed.py`**: Script for real-time sensor monitoring

### Launcher Scripts

- **`launch_apriltag_controller.py`**: Main launcher for the AprilTag controller with configuration options

## How to Use

### Basic Usage

```bash
# Launch the main AprilTag recognition system
python launch_apriltag_uart.py

# Launch in test mode (no camera required)
python launch_apriltag_uart.py --test

# Launch the alternative controller implementation
python launch_apriltag_controller.py
```

### Advanced Options

```bash
# Set custom motor speeds
python launch_apriltag_uart.py --max-speed 200 --min-speed 120

# Specify a serial port
python launch_apriltag_uart.py --port /dev/ttyACM1

# Enable verbose mode for detailed logging
python launch_apriltag_uart.py --verbose
```

## Direction Codes

The system uses standardized direction codes for omnidirectional movement:

- **0 (DIR_STOP)**: Stop movement
- **1 (DIR_FORWARD)**: Move forward
- **2 (DIR_BACKWARD)**: Move backward
- **3 (DIR_TURN_LEFT)**: Arc turn left
- **4 (DIR_TURN_RIGHT)**: Arc turn right
- **5 (DIR_ROTATE_LEFT)**: Rotate left in place
- **6 (DIR_ROTATE_RIGHT)**: Rotate right in place
- **7 (DIR_SLIDE_LEFT)**: Lateral movement to the left
- **8 (DIR_SLIDE_RIGHT)**: Lateral movement to the right
- **9 (DIR_DIAGONAL_FORWARD_LEFT)**: Diagonal movement forward-left
- **10 (DIR_DIAGONAL_FORWARD_RIGHT)**: Diagonal movement forward-right
- **11 (DIR_DIAGONAL_BACKWARD_LEFT)**: Diagonal movement backward-left
- **12 (DIR_DIAGONAL_BACKWARD_RIGHT)**: Diagonal movement backward-right

## Arduino Integration

The system is designed to work with the `integrated_movement.ino` Arduino sketch, which expects UART communication with message framing using `<` and `>` markers.

## Troubleshooting

If you encounter communication issues:

1. Check that your Arduino is connected and powered on
2. Try specifying the correct port with `--port`
3. Enable verbose mode with `--verbose` for detailed logs
4. Check the Arduino sketch has the correct baud rate (115200)
5. Ensure message framing is consistent (both use `<` and `>` markers)
