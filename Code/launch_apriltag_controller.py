#!/usr/bin/env python3
"""
AprilTag UART Controller Launcher

This script launches the dedicated AprilTag UART Controller implementation.
"""

import os
import sys
import argparse
import subprocess
import time
import platform


def main():
    parser = argparse.ArgumentParser(
        description='Launch AprilTag UART Controller')
    parser.add_argument('--max-speed', type=int, default=100,
                        help='Maximum motor speed (100-255)')
    parser.add_argument('--min-speed', type=int, default=50,
                        help='Minimum motor speed (50-max_speed)')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port (auto-detected if not specified)')
    parser.add_argument('--charging-time', type=str, default="19:30",
                        help='Time to go to charging station, format HH:MM (default: 19:30)')
    parser.add_argument('--work-time', type=str, default="09:00",
                        help='Time to go to work location, format HH:MM (default: 09:00)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera index for systems with multiple cameras')
    args = parser.parse_args()

    # Build the command
    script = "apriltag_uart_controller.py"
    cmd = [sys.executable, script]

    if args.max_speed:
        cmd.extend(["--max-speed", str(args.max_speed)])

    if args.min_speed:
        cmd.extend(["--min-speed", str(args.min_speed)])

    if args.port:
        cmd.extend(["--port", args.port])

    if args.charging_time:
        cmd.extend(["--charging-time", args.charging_time])

    if args.work_time:
        cmd.extend(["--work-time", args.work_time])

    if args.verbose:
        cmd.append("--verbose")

    if args.camera:
        cmd.extend(["--camera", str(args.camera)])

    # Print info
    print(f"Starting dedicated AprilTag UART Controller...")
    print(f"Max speed: {args.max_speed}")
    print(f"Min speed: {args.min_speed}")
    if args.port:
        print(f"Serial port: {args.port}")
    else:
        print("Serial port: Auto-detect")
    print(f"Charging time: {args.charging_time}")
    print(f"Work time: {args.work_time}")
    print(f"Verbose mode: {'ON' if args.verbose else 'OFF'}")

    try:
        # Run the command
        process = subprocess.Popen(cmd)
        process.wait()
    except KeyboardInterrupt:
        print("Launcher interrupted by user.")
        process.terminate()
    except Exception as e:
        print(f"Error launching script: {e}")
    finally:
        print("Launcher exiting.")


if __name__ == "__main__":
    main()
