"""
Stream lidar scans to stdout at the requested rate.
The requested rate can be arbitrarily fast as the
returned scans contains the most recently streamed
measures plus prior measurements to complete the scan.

insort_right() and bisect_right() copied from cpython
under this license https://github.com/python/cpython/blob/main/LICENSE
"""

import argparse
import json
from math import cos, sin, pi
import sys
import time

from adafruit_rplidar import RPLidar

           
if __name__ == "__main__":
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--rate", type=float, default=20, help = "Number of scans per second")
    parser.add_argument("-n", "--number", type=int, default=40, help = "Number of scans to collect")
    parser.add_argument("-a", "--min-angle", type=float, default=0, help="Minimum angle in degress (inclusive) to save")
    parser.add_argument("-A", "--max-angle", type=float, default=360, help="Maximum angle in degrees (inclusive) to save")
    parser.add_argument("-d", "--min-distance", type=float, default=sys.float_info.min, help="Minimum distance (inclusive) to save")
    parser.add_argument("-D", "--max-distance", type=float, default=sys.float_info.max, help="Maximum distance (inclusive) to save")
    parser.add_argument("-f", "--forward-angle", type=float, default=0.0, help="Forward angle - the angle facing 'forward'")
    parser.add_argument("-s", "--spin-reverse", action='store_true', help="Reverse 'spin' of lidar")

    # Read arguments from command line
    args = parser.parse_args()
    
    help = []
    if args.rate < 1:
        help.append("-r/--rate: must be >= 1.")
        
    if args.number < 1:
        help.append("-n/--number: must be >= 1.")
        
    if args.min_distance < 0:
        help.append("-d/--min_distance must be >= 0")

    if args.max_distance <= 0:
        help.append("-D/--max_distance must be > 0")
        
    if args.min_angle < 0 or args.min_angle > 360:
        help.append("-a/--min_angle must be 0 <= min_angle <= 360")

    if args.max_angle <= 0 or args.max_angle > 360:
        help.append("-A/--max_angle must be 0 < max_angle <= 360")
      
    if args.forward_angle < 0 or args.forward_angle > 360:
        help.append("-f/--forward_angle must be 0 <= forward_angle <= 360")

    if len(help) > 0:
        parser.print_help()
        for h in help:
            print("  " + h)
        sys.exit(1)
        
    
    lidar = None
    lidar_info = {}
    lidar_scans = []
    try:
        # Setup the RPLidar
        PORT_NAME = '/dev/ttyUSB0'
        lidar = RPLidar(None, PORT_NAME, timeout=3)
        
        seconds_per_scan = 1.0 / args.rate

        print("[")
        
        measurement_buffer = []
        measurement_count = 0  # number of measurements in the scan
        measurement_index = 0  # index of next measurement in the scan
        scan_time = time.time() + seconds_per_scan  # next time to emit a scans
        scan_count = 0
        overall_index = 0
        full_scan_count = 0
        full_scan_index = 0
        for new_scan, quality, angle, distance in lidar.iter_measurements():
            # check for start of new scan
            if new_scan:
                full_scan_count += 1
                full_scan_index = 0
                measurement_count = measurement_index  # number of points in this full scan
                measurement_index = 0   # start filling in next scan
                
            if scan_count > args.number:
                break
            
            # rplidar spins clockwise, but we want angles to increase counter-clockwise
            if args.spin_reverse:
                angle = (360.0 - (angle % 360.0)) % 360.0
            
            # adjust so zero degrees is 'forward'
            angle = (angle - args.forward_angle + 360.0) % 360.0
        
            now = time.time()
            if angle >= args.min_angle and angle <= args.max_angle:
                if distance >= args.min_distance and distance <= args.max_distance:
                    # convert to (x,y) point on plane
                    radians = angle * pi / 180.0
                    x = distance * cos(radians)
                    y = distance * sin(radians)
                    
                    measurement = {
                        "scan": full_scan_count,
                        "index": full_scan_index,
                        "overall": overall_index,
                        "time": now,
                        "distance": distance,
                        "angle": angle,
                        "x": x,
                        "y": y
                    }
                    
                    if measurement_index >= len(measurement_buffer):
                        measurement_buffer.append(measurement)
                        measurement_count = measurement_index + 1
                    else:
                        measurement_buffer[measurement_index] = measurement
                    measurement_index += 1
                    full_scan_index += 1
                    overall_index += 1
            
            if now >= scan_time:
                # emit the scan
                scan_count += 1
                scan_time = now + seconds_per_scan
                print(json.dumps({"scan": measurement_buffer[:measurement_count]}, indent=2) + ",")
                                    
    except KeyboardInterrupt:
        print('Stopping early.')
    except Exception as e:
        print(e)
        exit(1)
    finally:
        if lidar is not None:
            print("]")
            lidar.stop()
            lidar.disconnect()

