#!/usr/bin/env python3
"""
Communication Test for Arduino - Monitors sensor data with improved error handling
This script tests the communication between Raspberry Pi and Arduino,
focusing on receiving sensor data from the Arduino.
"""

import time
import serial
import argparse
import sys

# Default settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200

def monitor_sensor_data(port, baud_rate, duration=60):
    """
    Monitor sensor data coming from Arduino for the specified duration
    
    Args:
        port (str): Serial port to connect to
        baud_rate (int): Baud rate for serial communication
        duration (int): Duration in seconds to monitor (0 for indefinite)
    """
    try:
        # Connect to Arduino
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to Arduino on {port} at {baud_rate} baud")
        ser.reset_input_buffer()
        
        # Send a TEST command to verify connection
        print("Sending TEST command to Arduino...")
        ser.write(b'TEST\r\n')
        
        # Wait for response
        time.sleep(0.5)
        
        # Read and display response
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Arduino response:\n{response}")
        
        # Monitor for sensor data
        print("\nMonitoring for sensor data...")
        print("Press Ctrl+C to stop\n")
        
        start_time = time.time()
        last_data_time = 0
        
        # Setup for statistics
        data_count = 0
        data_intervals = []
        
        while duration == 0 or (time.time() - start_time) < duration:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        now = time.time()
                        
                        # Track data statistics
                        if last_data_time > 0:
                            interval = now - last_data_time
                            data_intervals.append(interval)
                        
                        last_data_time = now
                        data_count += 1
                        
                        # Process line - look for sensor data format
                        if line.startswith("SENS:"):
                            # Parse sensor data
                            print(f"[{time.time() - start_time:.1f}s] Sensor data: {line}")
                            
                            # Extract and format the values
                            try:
                                parts = line[5:].split(',')  # Remove "SENS:" prefix and split by comma
                                values = {}
                                for part in parts:
                                    if ':' in part:  # Make sure part contains a colon
                                        sensor, value = part.split(':')
                                        try:
                                            values[sensor] = float(value)
                                        except ValueError:
                                            # If we can't convert to float, mark as invalid
                                            values[sensor] = None
                                            print(f"  Warning: Invalid value for {sensor}: '{value}'")
                                
                                # Only print if we have at least some values
                                if values:
                                    # Print formatted values with safe handling of None values
                                    print("  Distances:")
                                    print(f"    Front Left: {values.get('FL', 'N/A') if values.get('FL') is not None else 'N/A'}cm, "
                                          f"Front: {values.get('F', 'N/A') if values.get('F') is not None else 'N/A'}cm, "
                                          f"Front Right: {values.get('FR', 'N/A') if values.get('FR') is not None else 'N/A'}cm")
                                    print(f"    Back Left: {values.get('BL', 'N/A') if values.get('BL') is not None else 'N/A'}cm, "
                                          f"Back: {values.get('B', 'N/A') if values.get('B') is not None else 'N/A'}cm, "
                                          f"Back Right: {values.get('BR', 'N/A') if values.get('BR') is not None else 'N/A'}cm")
                                else:
                                    print("  No valid sensor values found in message")
                            except Exception as e:
                                print(f"Error parsing sensor data: {e}")
                                print(f"Raw data: {line}")
                        else:
                            # For other messages, just print them
                            print(f"[{time.time() - start_time:.1f}s] Arduino: {line}")
                except UnicodeDecodeError:
                    pass  # Ignore decode errors
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
        
        # Print statistics
        print("\nCommunication Statistics:")
        print(f"Total messages received: {data_count}")
        
        if data_intervals:
            avg_interval = sum(data_intervals) / len(data_intervals)
            print(f"Average interval between messages: {avg_interval:.3f}s")
            print(f"Expected sensor data interval: {0.5}s")
            print(f"Communication health: {'Good' if 0.4 < avg_interval < 0.6 else 'Check timing'}")
    
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")
    except serial.SerialException as e:
        print(f"\nSerial port error: {e}")
        return False
    finally:
        # Clean up
        if 'ser' in locals() and ser.is_open:
            # Send STOP command before disconnecting
            ser.write(b'STOP\r\n')
            time.sleep(0.1)
            ser.close()
            print("Serial connection closed")
    
    return True

def main():
    """Main function to parse arguments and run the test"""
    parser = argparse.ArgumentParser(description="Monitor sensor data from Arduino")
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--duration', type=int, default=60,
                        help='Duration in seconds to monitor (0 for indefinite, default: 60)')
    args = parser.parse_args()
    
    # Run the monitoring function
    success = monitor_sensor_data(args.port, args.baud, args.duration)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
