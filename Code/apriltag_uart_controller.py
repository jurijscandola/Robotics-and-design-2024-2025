#!/usr/bin/env python3
"""
AprilTag UART Controller

This module integrates AprilTag recognition with UART communication to control the robot.
It acts as a bridge between the AprilTag detection system and the Arduino's integrated movement controller.
"""

import cv2
import time
import os
import numpy as np
import sys
import signal
import datetime
from threading import Event, Timer
import argparse
from typing import Optional, Tuple, List
import logging

# Import the UART communication module
from uart_communication import UARTCommunicator, TagData, DIR_FORWARD, DIR_BACKWARD, DIR_TURN_LEFT, DIR_TURN_RIGHT, DIR_STOP, DIR_ROTATE_LEFT, DIR_ROTATE_RIGHT


def direction_to_str(direction):
    """Convert direction code to readable string"""
    return {
        DIR_STOP: "STOP",
        DIR_FORWARD: "FORWARD",
        DIR_BACKWARD: "BACKWARD",
        DIR_TURN_LEFT: "TURN LEFT",
        DIR_TURN_RIGHT: "TURN RIGHT",
        DIR_ROTATE_LEFT: "ROTATE LEFT",
        DIR_ROTATE_RIGHT: "ROTATE RIGHT"
    }.get(direction, "UNKNOWN")


# Check if running on Raspberry Pi
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# === Configuration ===
# Camera settings
CAMERA_WIDTH = 1640
CAMERA_HEIGHT = 1232
FOCAL_LENGTH_PX = 1000
TAG_SIZE_MM = 150
TAG_SIZE_CM = TAG_SIZE_MM / 10

# Camera intrinsic parameters
CAM_FX = 1000.0
CAM_FY = 1000.0
CAM_CX = CAMERA_WIDTH / 2
CAM_CY = CAMERA_HEIGHT / 2

# Arduino communication
ARDUINO_ENABLED = True
DEFAULT_SERIAL_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_MAX_SPEED = 200
DEFAULT_MIN_SPEED = 100

# Detection settings
VERBOSE = False
DETECTION_INTERVAL = 0.1

# Navigation parameters
CENTER_TOLERANCE = 0.1
DISTANCE_THRESHOLD = 30

# Time-based task scheduling
# Define tag IDs for special locations
CHARGING_BASE_TAG_ID = 1  # Assume tag ID 1 is the charging base
WORK_LOCATION_TAG_ID = 2  # Assume tag ID 2 is the work location

# Default schedule times (can be overridden via command line)
DEFAULT_CHARGING_TIME = "19:30"  # Robot goes to charging station at 7:30 PM
DEFAULT_WORK_TIME = "09:00"      # Robot goes to work location at 9:00 AM


# === Global state ===
shutdown_event = Event()


class AprilTagUARTController:
    """
    Controller that integrates AprilTag detection with UART communication
    to control the robot movement.
    """

    def __init__(self,
                 port: Optional[str] = None,
                 baud_rate: int = DEFAULT_BAUD_RATE,
                 max_speed: int = DEFAULT_MAX_SPEED,
                 min_speed: int = DEFAULT_MIN_SPEED,
                 verbose: bool = False,
                 charging_time: str = DEFAULT_CHARGING_TIME,
                 work_time: str = DEFAULT_WORK_TIME,
                 use_three_wheels: bool = False):
        """Initialize the controller with communication parameters"""
        self.port = port
        self.baud_rate = baud_rate
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.verbose = verbose
        self.communicator = None
        self.camera = None
        self.detector = None
        self.running = False
        self.use_three_wheels = use_three_wheels

        # Time-based scheduling attributes
        self.charging_time = charging_time
        self.work_time = work_time
        self.scheduled_task_timer = None
        self.current_target_tag_id = None
        self.search_mode = False
        self.search_pattern_index = 0
        self.last_scheduled_check = 0

        if verbose:
            logger.setLevel(logging.DEBUG)

    def calculate_desired_speed(self, distance_cm: float, base_speed: int) -> int:
        """Calculate desired speed based on distance to target

        This calculates the speed the robot should try to move at to reach the target.
        The Arduino will apply additional safety scaling if obstacles are detected.
        """
        if distance_cm < DISTANCE_THRESHOLD:
            return 0  # Stop when too close to target

        # Up to 2 meters away, scale speed linearly (farther = faster)
        max_distance = 200  # cm
        speed_factor = min(distance_cm / max_distance, 1.0)

        # Calculate speed as percentage of range between min and max
        desired_speed = int(self.min_speed + (self.max_speed -
                                              self.min_speed) * speed_factor)

        # Ensure speed doesn't exceed the max or go below the min
        return max(self.min_speed, min(self.max_speed, desired_speed))

    def setup(self) -> bool:
        """
        Initialize hardware and communication

        Returns:
            bool: True if setup successful, False otherwise
        """
        # Initialize UART communication
        logger.info("Setting up UART communication...")
        self.communicator = UARTCommunicator(
            port=self.port,
            baud_rate=self.baud_rate,
            debug=self.verbose
        )

        if not self.communicator.connected:
            logger.info("Auto-detection of Arduino port...")
            if not self.communicator.connect():
                logger.error(
                    "Failed to connect to Arduino. Check connections and try again.")
                return False        # Set motor speed parameters
        logger.info(
            f"Setting motor speeds - MAX: {self.max_speed}, MIN: {self.min_speed}")
        self.communicator.set_speed(self.max_speed, self.min_speed)

        # Configure wheel mode (2-wheel or 3-wheel)
        logger.info(
            f"Setting wheel mode: {'THREE_WHEEL' if self.use_three_wheels else 'TWO_WHEEL'}")
        self.communicator.set_wheel_mode(self.use_three_wheels)

        # Initialize camera if on Raspberry Pi
        if IS_RASPBERRY_PI:
            logger.info("Setting up camera...")
            self.camera = self.setup_camera()
            if not self.camera:
                logger.error("Failed to initialize camera.")
                return False

            # Initialize AprilTag detector
            logger.info("Setting up AprilTag detector...")
            self.detector = Detector(
                families="tag36h11",
                nthreads=4,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=True,
                decode_sharpening=0.5,
                debug=False
            )
        else:
            logger.warning(
                "Not running on Raspberry Pi - camera features unavailable")

        logger.info("Setup complete!")
        return True

    def setup_camera(self):
        """Initialize the Raspberry Pi camera with optimal settings for AprilTag detection"""
        if not IS_RASPBERRY_PI:
            return None

        try:
            # Initialize camera
            picam = Picamera2()

            # Configure camera with settings optimized for tag detection
            config = picam.create_preview_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT),
                      "format": "BGR888"},
                controls={
                    "FrameRate": 15,
                    "AwbEnable": True,
                    "ExposureTime": 8000,
                    "AnalogueGain": 4.0,
                    "Sharpness": 15.0,
                    "Contrast": 2.0,
                    "Brightness": 0.0,
                    "NoiseReductionMode": 0,
                    "AwbMode": 1
                }
            )

            picam.set_controls({"NoiseReductionMode": 0})
            picam.configure(config)
            picam.start()
            time.sleep(0.5)

            try:
                picam.set_controls({
                    "FrameDurationLimits": (33333, 66666)
                })
            except Exception:
                pass  # Ignore if not supported

            logger.info(
                f"Camera initialized with resolution {CAMERA_WIDTH}x{CAMERA_HEIGHT}")

            return picam

        except Exception as e:
            logger.error(f"Camera setup failed: {e}")
            try:
                import subprocess
                subprocess.run(['sudo', 'pkill', '-f', 'libcamera'],
                               stderr=subprocess.DEVNULL)
            except:
                pass
            return None

    def preprocess_image(self, frame):
        """Advanced image preprocessing for better tag detection"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply advanced adaptive histogram equalization
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
        gray = clahe.apply(gray)

        # Apply bilateral filter
        gray = cv2.bilateralFilter(gray, 5, 75, 75)

        # Enhance contrast
        gray = cv2.convertScaleAbs(gray, alpha=1.5, beta=15)

        # Apply edge enhancement
        kernel = np.array([[-1, -1, -1], [-1, 9.5, -1], [-1, -1, -1]])
        gray = cv2.filter2D(gray, -1, kernel)

        return gray

    def estimate_tag_size(self, corners):
        """Calculate tag size in pixels based on corner positions with robust metrics"""
        # Calculate all side lengths
        side_lengths = []
        for i in range(4):
            side = np.linalg.norm(corners[i] - corners[(i+1) % 4])
            side_lengths.append(side)

        # Calculate diagonal lengths
        diagonal1 = np.linalg.norm(corners[0] - corners[2])
        diagonal2 = np.linalg.norm(corners[1] - corners[3])

        # Verify if the shape is roughly square
        sides_std = np.std(side_lengths)
        sides_mean = np.mean(side_lengths)
        diag_ratio = max(diagonal1, diagonal2) / min(diagonal1, diagonal2)

        # Handle distorted tags
        if sides_std / sides_mean > 0.2 or diag_ratio > 1.3:
            estimated_side = np.median(side_lengths)
            estimated_side_from_diag = (
                diagonal1 + diagonal2) / (2 * np.sqrt(2))
            return 0.7 * estimated_side + 0.3 * estimated_side_from_diag
        else:
            return sides_mean

    def calculate_distance(self, tag_size_px):
        """Calculate distance to AprilTag using a more accurate camera model"""
        distance_mm = (CAM_FX * TAG_SIZE_MM) / tag_size_px
        distance_cm = distance_mm / 10.0

        # Apply calibration correction
        if distance_cm < 50:
            return distance_cm * 1.05
        elif distance_cm < 150:
            return distance_cm * 1.02
        else:
            return distance_cm * 0.98

    def get_direction_and_speed(self, detection, frame_width):
        """Determine movement direction and speed based on tag position and size"""
        center_x = detection.center[0]
        tag_size_px = self.estimate_tag_size(detection.corners)
        distance_cm = self.calculate_distance(tag_size_px)

        # Calculate frame center and tolerance zone
        frame_center = frame_width / 2
        tolerance = frame_width * CENTER_TOLERANCE

        # Calculate desired speed based on distance to tag
        speed = self.calculate_desired_speed(distance_cm, self.max_speed)

        # Determine direction
        if distance_cm < DISTANCE_THRESHOLD:
            direction = DIR_STOP
            speed = 0
        else:
            if center_x < frame_center - tolerance:
                direction = DIR_TURN_LEFT
                # Calculate proportional speed based on how far off-center
                offset_ratio = min(
                    1.0, abs(center_x - frame_center) / (frame_width/4))
                turn_factor = 0.25 + (0.25 * offset_ratio)  # 25-50% of speed
                speed = max(self.min_speed, int(speed * turn_factor))
            elif center_x > frame_center + tolerance:
                direction = DIR_TURN_RIGHT
                # Reduce speed for turning - LOWER VALUE

                # Reduced from 0.7 to 0.4
                speed = max(self.min_speed, int(speed * 0.4))
            else:
                direction = DIR_FORWARD

        if self.verbose:
            logger.debug(
                f"Distance: {distance_cm:.1f}cm, Direction: {direction_to_str(direction)}, Speed: {speed}")

        return direction, speed

    def process_frame(self, frame):
        """Process a single frame to detect AprilTags and control robot"""
        if not self.detector:
            logger.error("AprilTag detector not initialized")
            return

        # Preprocess image and detect tags
        gray = self.preprocess_image(frame)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(CAM_FX, CAM_FY, CAM_CX, CAM_CY),
            tag_size=TAG_SIZE_CM
        )

        # Handle search mode
        if self.search_mode and self.current_target_tag_id is not None:
            matching_tags = [
                d for d in detections if d.tag_id == self.current_target_tag_id]
            if matching_tags:
                target_detection = matching_tags[0]
                direction, speed = self.get_direction_and_speed(
                    target_detection, frame.shape[1])

                # Calculate actual distance to tag for obstacle avoidance
                tag_size_px = self.estimate_tag_size(target_detection.corners)
                distance_cm = self.calculate_distance(tag_size_px)

                tag_data = TagData(
                    tag_id=target_detection.tag_id,
                    speed=speed,
                    direction=direction,
                    distance=distance_cm  # Include actual physical distance
                )
                self.communicator.send_tag_data(tag_data)
                return
            elif not self.scheduled_task_timer or not self.scheduled_task_timer.is_alive():
                self.execute_search_pattern()
                return

        # Regular tag detection processing
        if not detections:
            if self.verbose:
                logger.info("No tags detected")
            self.communicator.send_stop()
            return

        # Find closest tag and process it
        best_detection = max(
            detections, key=lambda d: self.estimate_tag_size(d.corners))
        direction, speed = self.get_direction_and_speed(
            best_detection, frame.shape[1])

        # Calculate actual distance to tag for obstacle avoidance
        tag_size_px = self.estimate_tag_size(best_detection.corners)
        distance_cm = self.calculate_distance(tag_size_px)

        tag_data = TagData(
            tag_id=best_detection.tag_id,
            speed=speed,
            direction=direction,
            distance=distance_cm  # Include actual physical distance
        )
        self.communicator.send_tag_data(tag_data)

    def run(self):
        """Run the main detection and control loop"""
        if not IS_RASPBERRY_PI:
            logger.error(
                "This module requires a Raspberry Pi to run in camera mode")
            return

        if not self.camera or not self.detector or not self.communicator:
            logger.error(
                "Controller not properly initialized. Run setup() first.")
            return

        self.running = True
        logger.info("Starting AprilTag detection and robot control loop...")

        try:
            last_time = time.time()

            while self.running and not shutdown_event.is_set():
                # Capture frame
                frame = self.camera.capture_array()

                # Process at fixed interval for stability
                current_time = time.time()
                if current_time - last_time >= DETECTION_INTERVAL:
                    self.process_frame(frame)
                    last_time = current_time

                # Check for keyboard interrupt
                if cv2.waitKey(1) == ord('q'):
                    break

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Error in detection loop: {e}")
        finally:
            self.stop()

    def stop(self):
        """Stop the controller and clean up resources"""
        self.running = False

        logger.info("Shutting down...")

        # Stop the robot
        if self.communicator:
            logger.info("Sending stop command to robot")
            self.communicator.send_stop()

        # Release camera resources
        if IS_RASPBERRY_PI and self.camera:
            logger.info("Releasing camera")
            self.camera.close()

        logger.info("Shutdown complete")

    def schedule_task(self, tag_id: int, action: str, time_str: str):
        """Schedule a task for the robot based on time and tag ID"""
        # Convert time string to seconds since midnight
        try:
            target_time = datetime.datetime.strptime(
                time_str, "%H:%M").time()
            now = datetime.datetime.now()
            scheduled_datetime = datetime.datetime.combine(
                now.date(), target_time)

            # If the scheduled time is in the past, move to the next day
            if scheduled_datetime < now:
                scheduled_datetime += datetime.timedelta(days=1)

            delay = (scheduled_datetime - now).total_seconds()

            logger.info(
                f"Task scheduled: {action} at {time_str} (in {delay} seconds)")

            # Cancel any existing timer
            if self.scheduled_task_timer:
                self.scheduled_task_timer.cancel()

            # Schedule the task
            self.scheduled_task_timer = Timer(
                delay, self.execute_scheduled_task, [tag_id, action])
            self.scheduled_task_timer.start()

        except Exception as e:
            logger.error(f"Failed to schedule task: {e}")

    def execute_scheduled_task(self, tag_id: int, action: str):
        """Execute the scheduled task"""
        logger.info(f"Executing scheduled task: {action} for tag {tag_id}")

        if action == "charge":
            self.go_to_charging_station()
        elif action == "work":
            self.go_to_work_location()
        else:
            logger.warning(f"Unknown action: {action}")

    def go_to_charging_station(self):
        """Navigate to the charging station"""
        logger.info("Navigating to charging station...")
        self.search_mode = True
        self.current_target_tag_id = CHARGING_BASE_TAG_ID
        self.search_pattern_index = 0

    def go_to_work_location(self):
        """Navigate to the work location"""
        logger.info("Navigating to work location...")
        self.search_mode = True
        self.current_target_tag_id = WORK_LOCATION_TAG_ID
        self.search_pattern_index = 0

    def check_scheduled_tasks(self):
        """Check if any scheduled tasks need to be executed based on current time"""
        now = datetime.datetime.now()
        current_time_str = now.strftime("%H:%M")

        # Only check once per minute to avoid constantly triggering
        if time.time() - self.last_scheduled_check < 60:
            return

        self.last_scheduled_check = time.time()

        # Log status every 15 minutes for debugging
        if now.minute % 15 == 0 and now.second < 10:
            self.log_schedule_status()

        # Check for charging time
        if current_time_str == self.charging_time:
            logger.info(
                f"Scheduled task: Time to go to charging station (Tag ID: {CHARGING_BASE_TAG_ID})")
            self.start_scheduled_navigation(CHARGING_BASE_TAG_ID)

        # Check for work time
        elif current_time_str == self.work_time:
            logger.info(
                f"Scheduled task: Time to go to work location (Tag ID: {WORK_LOCATION_TAG_ID})")
            self.start_scheduled_navigation(WORK_LOCATION_TAG_ID)

    def log_schedule_status(self):
        """Log the current schedule status for debugging purposes"""
        now = datetime.datetime.now()
        now_time = now.time()

        # Parse schedule times
        charging_time = datetime.datetime.strptime(
            self.charging_time, "%H:%M").time()
        work_time = datetime.datetime.strptime(self.work_time, "%H:%M").time()

        # Calculate time until next tasks
        charging_dt = datetime.datetime.combine(now.date(), charging_time)
        if now_time > charging_time:
            charging_dt += datetime.timedelta(days=1)

        work_dt = datetime.datetime.combine(now.date(), work_time)
        if now_time > work_time:
            work_dt += datetime.timedelta(days=1)

        time_to_charging = (charging_dt - now).total_seconds() / 3600  # hours
        time_to_work = (work_dt - now).total_seconds() / 3600  # hours

        logger.info(
            f"Schedule status - Current time: {now_time.strftime('%H:%M')}")
        logger.info(
            f"  - Charging time: {self.charging_time} (in {time_to_charging:.1f} hours)")
        logger.info(
            f"  - Work time: {self.work_time} (in {time_to_work:.1f} hours)")

    def start_scheduled_navigation(self, target_tag_id):
        """Start searching for a specific tag for scheduled navigation"""
        self.current_target_tag_id = target_tag_id
        self.search_mode = True
        self.search_pattern_index = 0
        logger.info(f"Starting search for Tag ID {target_tag_id}")

    def execute_search_pattern(self):
        """Execute a search pattern to find the target tag if not immediately visible"""
        # Simple search pattern: rotate in place to scan surroundings
        search_patterns = [
            (DIR_ROTATE_RIGHT, 5),
            (DIR_STOP, 1),
            (DIR_ROTATE_RIGHT, 5),
            (DIR_STOP, 1),
            (DIR_TURN_RIGHT, 5),
            (DIR_STOP, 1),
            (DIR_TURN_LEFT, 20),
            (DIR_STOP, 1),
            (DIR_BACKWARD, 5),
            (DIR_STOP, 1),
            (DIR_TURN_RIGHT, 10),
            (DIR_STOP, 1),
            (DIR_BACKWARD, 10),
            (DIR_STOP, 1),
            (DIR_ROTATE_LEFT, 15),
            (DIR_STOP, 1)
        ]

        # If we've completed the pattern without finding the tag, reset and try again
        if self.search_pattern_index >= len(search_patterns):
            self.search_pattern_index = 0
            logger.info("Search pattern complete, resetting...")
            return

        # Get next search movement
        direction, duration = search_patterns[self.search_pattern_index]

        # Create tag data for the search movement
        tag_data = TagData(
            tag_id=self.current_target_tag_id if self.current_target_tag_id else 0,
            distance=100.0,  # Default distance for search movements
            direction=direction,
            speed=self.min_speed  # Always use minimum speed for search pattern
        )

        logger.info(
            f"Search pattern step {self.search_pattern_index}: {direction_to_str(direction)}")
        self.communicator.send_tag_data(tag_data)

        # Schedule next step in the search pattern after this step completes
        self.search_pattern_index += 1
        self.scheduled_task_timer = Timer(
            duration, self._next_search_step)
        self.scheduled_task_timer.start()

        # Send movement command
        self.communicator.send_tag_data(tag_data)

        # Schedule the next step after the specified duration
        if self.scheduled_task_timer:
            self.scheduled_task_timer.cancel()

        self.scheduled_task_timer = Timer(
            duration,
            self._next_search_step
        )
        self.scheduled_task_timer.start()

    def _next_search_step(self):
        """Helper method to advance to the next search step"""
        self.search_pattern_index += 1
        if self.search_mode and self.current_target_tag_id is not None:
            self.execute_search_pattern()

    def update(self):
        """Update method to be called periodically"""
        if not self.running:
            return

        # Check and execute scheduled tasks
        self.check_scheduled_tasks()

        # If in search mode, perform the search pattern
        if self.search_mode:
            self.perform_search_pattern()

    def perform_search_pattern(self):
        """Perform the search pattern to find the target tag"""
        if self.current_target_tag_id is None:
            return

        # Implement a simple search pattern: square or circular
        if self.search_pattern_index < 4:
            # Move in a square pattern with reversed forward/backward
            # Changed FORWARD to BACKWARD and BACKWARD to FORWARD
            directions = [DIR_BACKWARD, DIR_TURN_LEFT,
                          DIR_FORWARD, DIR_TURN_RIGHT]
            direction = directions[self.search_pattern_index]

            # Use the correct method to send movement commands with reduced speed
            tag_data = TagData(
                tag_id=self.current_target_tag_id,
                speed=int(self.min_speed * 0.7),  # Use 70% of minimum speed
                direction=direction,
                # Default search distance (far enough to not cause issues)
                distance=100.0
            )
            self.communicator.send_tag_data(tag_data)

            time.sleep(1)
            self.search_pattern_index += 1
        else:
            # Search pattern complete, stop and reset
            self.communicator.send_stop()
            self.search_mode = False
            self.search_pattern_index = 0
            logger.info("Search pattern complete")


def signal_handler(sig, frame):
    """Handle shutdown signals"""
    logger.info("Shutdown signal received")
    shutdown_event.set()


def main():
    """Main function to run the controller from command line"""
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='AprilTag Detection with UART Robot Control')
    parser.add_argument('--port', type=str, default=None,
                        help=f'Serial port (default: auto-detect)')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')

    parser.add_argument('--max-speed', type=int, default=DEFAULT_MAX_SPEED,
                        help=f'Maximum motor speed (50-255) (default: {DEFAULT_MAX_SPEED})')
    parser.add_argument('--min-speed', type=int, default=DEFAULT_MIN_SPEED,
                        help=f'Minimum motor speed (30-max_speed) (default: {DEFAULT_MIN_SPEED})')
    parser.add_argument('--charging-time', type=str, default=DEFAULT_CHARGING_TIME,
                        help=f'Time to go to charging station, format HH:MM (default: {DEFAULT_CHARGING_TIME})')
    parser.add_argument('--work-time', type=str, default=DEFAULT_WORK_TIME,
                        help=f'Time to go to work location, format HH:MM (default: {DEFAULT_WORK_TIME})')
    parser.add_argument('--three-wheels', action='store_true',
                        help='Use 3-wheel configuration for omnidirectional movement (default: 2-wheel)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')
    args = parser.parse_args()

    # Create and start the controller
    controller = AprilTagUARTController(
        port=args.port,
        baud_rate=args.baud,
        max_speed=args.max_speed,
        min_speed=args.min_speed,
        verbose=args.verbose,
        charging_time=args.charging_time,
        work_time=args.work_time,
        use_three_wheels=args.three_wheels
    )

    if controller.setup():
        controller.run()
    else:
        logger.error("Failed to set up controller. Exiting.")
        sys.exit(1)


if __name__ == "__main__":
    main()
