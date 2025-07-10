# Localization and Movement Module (ILMModule)

A robust localization and navigation system for an omnidirectional robot featuring AprilTag-based positioning, sensor fusion, I2C coordination, and advanced movement control with UART communication.

## Features

- **Omnidirectional Movement Control**

  - Holonomic drive system with three-wheel configuration
  - Smooth acceleration and deceleration with direction preservation
  - Precise position and orientation control
  - Maximum safe speed: 0.5 m/s
  - Six distinct movement types including rotation in place

- **Advanced Localization**

  - Sensor fusion from multiple sources
  - High-resolution encoder readings (1120 ticks/revolution)
  - AprilTag-based absolute positioning
  - Position accuracy: ±5mm in controlled conditions
  - Real-time positioning feedback with distance-based control

- **Multiple Operation Modes**

  - Manual control via direct commands
  - Autonomous navigation with path planning
  - AprilTag following with distance control
  - Automated docking/charging
  - Emergency stop functionality
  - I2C coordination with master module

- **Obstacle Detection**
  - 6x HC-SR04 ultrasonic sensors
  - Real-time obstacle avoidance with configurable thresholds
  - Social navigation features
  - Multi-level safety checks
  - Dynamic speed adjustment based on obstacle proximity

## System Architecture

### Hardware Components

- 3x JGB37-520 Encoder DC motors (178 RPM, 1.8 Nm torque)
- 3x BTS7960 43A H-bridge motor drivers
- 6x HC-SR04 ultrasonic sensors
- Raspberry Pi 4 (for vision processing & high-level control)
- Elegoo Mega 2560 (for motor control and sensor fusion)

### Software Components

1. **AprilTag Recognition (Python)**

   - Camera-based tag detection using OpenCV and pupil_apriltags
   - Real-time pose estimation and distance calculation
   - 30 FPS detection rate with optimization
   - Support for multiple tag families and sizes

2. **UART Communication Layer (Python)**

   - High-speed serial protocol (115200 baud)
   - Message framing with start/end markers
   - Error handling and retry mechanisms
   - Standardized message formats
   - Auto-detection of serial ports

3. **Localization System (Arduino)**

   - Encoder-based odometry with high precision
   - Position and orientation tracking
   - 20Hz control loop frequency
   - I2C slave capability for master coordination

4. **Movement Control (Arduino)**
   - Fine-grained motor control with PWM
   - Dynamic speed adjustment based on conditions
   - Advanced obstacle avoidance logic
   - Smooth deceleration system
   - Emergency stop handling

## Communication Protocol

### UART Message Formats

- Tag Data: `<TAG:id,distance,direction>`
- Movement Command: `<MOVE:direction,speed>`
- Direct Motor Control: `<MCTL:left_speed,right_speed,back_speed>`
- Speed Setting: `<SPEED:max_speed,min_speed>`
- Sensor Request: `<SENS>`
- Connection Test: `<PING>`
- Stop Command: `<STOP>`
- I2C Master Command: `<I2CM:command>`

### Direction Codes

- `0` = STOP
- `1` = FORWARD
- `2` = BACKWARD
- `3` = LEFT (Arc turn)
- `4` = RIGHT (Arc turn)
- `5` = ROTATE LEFT (in place)
- `6` = ROTATE RIGHT (in place)
- `7` = SLIDE LEFT (lateral)
- `8` = SLIDE RIGHT (lateral)
- `9` = DIAGONAL FORWARD-LEFT
- `10` = DIAGONAL FORWARD-RIGHT
- `11` = DIAGONAL BACKWARD-LEFT
- `12` = DIAGONAL BACKWARD-RIGHT

## Getting Started

1. **Hardware Setup**

   - Connect motors to the BTS7960 drivers
   - Wire ultrasonic sensors to specified pins
   - Establish USB serial connection between Raspberry Pi and Arduino

2. **Software Installation**

   - Install required Python packages:
     ```
     pip install -r requirements.txt
     ```
   - Upload `integrated_movement.ino` to the Arduino Mega
   - Run camera calibration if needed:
     ```
     python camera_apriltag_test.py --calibrate
     ```
   - Test communication:
     ```
     python communication_test.py
     ```

3. **Running the System**

   - Start the main controller:
     ```
     python launch_apriltag_controller.py
     ```
   - For monitoring sensor data:
     ```
     python monitor_sensor_data_fixed.py
     ```
   - For manual motor testing:
     ```
     python motor_test.py
     ```

4. **Testing**
   - Run individual component tests using `Testing_Motors/Testing_Motors.ino` and `Testing_Sensors/Testing_Sensors.ino`
   - Verify sensor readings and motor operation
   - Check AprilTag detection accuracy with `test_apriltag_uart.py`
   - Test UART communication with `test_uart_communication.py`
   - Validate emergency stop functionality

## Component Details

### Smooth Deceleration System

The system implements a sophisticated smooth deceleration mechanism for improved safety and mechanical longevity:

- **Direction Tracking**: Each motor's direction (FORWARD, BACKWARD, STOP) is consistently tracked
- **Gradual Speed Reduction**: Progressive PWM reduction at 0.15 rate per cycle
- **Multi-level Safety Controls**:
  - Normal deceleration for standard stops (preserves direction information)
  - Emergency bypass for critical situations
  - Obstruction-triggered deceleration
- **Integration Points**:
  - Seamlessly integrates with obstacle avoidance system
  - Coordinates with dynamic speed calculations
  - Respects emergency stop signals
  - Preserves motor direction data across file boundaries

The deceleration system effectively prevents jerky stops, reduces mechanical stress, and improves passenger comfort while maintaining safety priorities.

## Project Structure

- **Core Components**:

  - `apriltag_recognition.py`: Vision processing and tag detection (updated for UART)
  - `apriltag_uart_controller.py`: Alternative implementation with UART communication
  - `uart_communication.py`: Robust UART communication library
  - `test_apriltag_uart.py`: Testing tool for UART communication
  - `test_uart_communication.py`: UART communication test utility
  - `camera_test.py`: Utility for testing camera functionality
  - `monitor_sensor_data_fixed.py`: Tool for monitoring sensor data

- **Launcher Scripts**:

  - `launch_apriltag_uart.py`: Easy-to-use launcher for AprilTag recognition system
  - `launch_apriltag_controller.py`: Launcher for alternative controller implementation

- **Documentation**:

  - `APRILTAG_UART_README.md`: User guide for the UART AprilTag system
  - `UART_COMMUNICATION_GUIDE.md`: Detailed guide for UART communication
  - `UART_INTEGRATION_SUMMARY.md`: Summary of the UART integration process

- **Arduino Code**:
  - `ILMMCodes/`: All Arduino implementations:
    - `integrated_movement/`: Main Arduino implementation for the robot
    - `apriltag_movement/`: Tag-following implementation
    - `localization/`: Sensor fusion and positioning
    - `basic_moveset/`: Basic movement functions
    - `fine_moveset/`: Fine-tuned movement control
- **Testing Tools**:

  - `Testing_Motors/`: Test code for motor functionality
  - `Testing_Sensors/`: Test code for sensor functionality

- **Project Documentation**:
  - `Robotics & Design_ Localization module anthology/`: Complete project documentation including:
    - Technical drawings
    - Hardware schematics
    - Project photos
    - LaTeX documentation

## Safety Features

- Multi-level obstacle detection
- Emergency stop functionality
- Motor current monitoring
- Sensor validation checks
- Fail-safe operation modes

## Performance Metrics

- Position accuracy: ±5mm in controlled conditions
- Heading accuracy: ±2 degrees
- Maximum safe speed: 0.5 m/s
- Control loop frequency: 20Hz
- Tag detection rate: 30 FPS
- Emergency stop response: <100ms

## Future Improvements

- Web-based control interface
- Real-time telemetry visualization
- Remote diagnostics capabilities
- User-friendly calibration interfaces
- Configuration backup/restore features

## Contributors

- Project documentation and technical drawings available in the `Robotics & Design_ Localization module anthology/` directory
- Hardware schematics provided in `Robotics & Design_ Localization module anthology/pfds/IL_schema.pdf`
- Obsolete and legacy files safely archived in `Obsolete_Files/`
