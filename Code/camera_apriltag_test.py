#!/usr/bin/env python3
"""
AprilTag Camera Test

This script tests the camera setup and AprilTag detection without robot movement.
Use this to validate that the camera is working properly and can detect AprilTags.
"""

import cv2
import time
import os
import numpy as np
import sys
import argparse
import logging

# Check if running on Raspberry Pi
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector
else:
    print("Error: This script must be run on a Raspberry Pi")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Camera settings
CAMERA_WIDTH = 1640
CAMERA_HEIGHT = 1232
TAG_SIZE_MM = 150

# Camera intrinsic parameters (can be tuned)
CAM_FX = 1000.0  # Focal length in x
CAM_FY = 1000.0  # Focal length in y  
CAM_CX = CAMERA_WIDTH / 2  # Principal point x
CAM_CY = CAMERA_HEIGHT / 2  # Principal point y

def preprocess_image(frame):
    """Advanced image preprocessing for better tag detection"""
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply adaptive histogram equalization
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
    gray = clahe.apply(gray)
    
    # Apply bilateral filter to reduce noise while preserving edges
    gray = cv2.bilateralFilter(gray, 5, 75, 75)
    
    # Enhance contrast
    gray = cv2.convertScaleAbs(gray, alpha=1.5, beta=15)
    
    return gray

def setup_camera():
    """Initialize the Raspberry Pi camera with optimal settings for AprilTag detection"""
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
        
        logger.info(f"Camera initialized with resolution {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
        
        return picam
    
    except Exception as e:
        logger.error(f"Camera setup failed: {e}")
        try:
            import subprocess
            subprocess.run(['sudo', 'pkill', '-f', 'libcamera'], stderr=subprocess.DEVNULL)
        except:
            pass
        return None

def main():
    parser = argparse.ArgumentParser(description='Test AprilTag detection with camera')
    parser.add_argument('--save', action='store_true', help='Save captured frames with detections')
    parser.add_argument('--debug', action='store_true', help='Show additional debug information')
    args = parser.parse_args()
    
    if args.debug:
        logger.setLevel(logging.DEBUG)
    
    # Initialize camera
    logger.info("Setting up camera...")
    camera = setup_camera()
    if not camera:
        logger.error("Failed to initialize camera. Exiting.")
        return
    
    # Initialize AprilTag detector
    logger.info("Setting up AprilTag detector...")
    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.5,
        debug=False
    )
    
    logger.info("Starting camera test. Press 'q' to quit, 's' to save current frame.")
    
    save_dir = "camera_captures"
    if args.save and not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    frame_count = 0
    detection_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Capture frame
            frame = camera.capture_array()
            frame_count += 1
            
            # Preprocess image
            gray = preprocess_image(frame)
            
            # Save original and preprocessed frames for analysis if debug is enabled
            if args.debug and frame_count % 30 == 0:  # Save every 30th frame
                timestamp = int(time.time())
                cv2.imwrite(f"{save_dir}/original_{timestamp}.jpg", frame)
                cv2.imwrite(f"{save_dir}/preprocessed_{timestamp}.jpg", gray)
            
            # Detect AprilTags
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(CAM_FX, CAM_FY, CAM_CX, CAM_CY),
                tag_size=TAG_SIZE_MM/10  # Convert to cm
            )
            
            # Process detections
            if detections:
                detection_count += 1
                for detection in detections:
                    # Draw tag outline
                    pts = detection.corners.astype(np.int32).reshape((-1, 1, 2))
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                    
                    # Display tag ID
                    center = tuple(detection.center.astype(np.int32))
                    cv2.putText(frame, f"ID: {detection.tag_id}", 
                                (center[0], center[1] - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
                    logger.info(f"Detected tag ID {detection.tag_id} at {center}")
                    
                    if hasattr(detection, 'pose_t'):
                        # Display distance estimate if pose is available
                        distance = np.linalg.norm(detection.pose_t) * 100  # Convert to cm
                        cv2.putText(frame, f"Dist: {distance:.1f} cm", 
                                    (center[0], center[1] + 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "No tags detected", (30, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Calculate and display FPS
            elapsed_time = time.time() - start_time
            fps = frame_count / elapsed_time
            cv2.putText(frame, f"FPS: {fps:.1f}", (frame.shape[1] - 120, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Display detection rate
            detection_rate = (detection_count / frame_count) * 100
            cv2.putText(frame, f"Detection rate: {detection_rate:.1f}%", 
                       (frame.shape[1] - 250, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Display the frame
            cv2.imshow("AprilTag Detection Test", frame)
            
            # Check for keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save current frame
                timestamp = int(time.time())
                filename = f"{save_dir}/captured_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                logger.info(f"Saved frame to {filename}")
            
            # Print stats every 5 seconds
            if int(elapsed_time) % 5 == 0 and int(elapsed_time) > 0:
                logger.info(f"Stats: FPS={fps:.1f}, Detection rate={detection_rate:.1f}%")
    
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    except Exception as e:
        logger.error(f"Error during camera test: {e}")
    finally:
        if camera:
            camera.close()
        cv2.destroyAllWindows()
        
        # Print final statistics
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        detection_rate = (detection_count / frame_count) * 100 if frame_count > 0 else 0
        
        logger.info("=== Camera Test Summary ===")
        logger.info(f"Total frames processed: {frame_count}")
        logger.info(f"Average FPS: {fps:.1f}")
        logger.info(f"Frames with detections: {detection_count} ({detection_rate:.1f}%)")
        logger.info(f"Test duration: {elapsed_time:.1f} seconds")
        logger.info("=========================")

if __name__ == "__main__":
    main()
