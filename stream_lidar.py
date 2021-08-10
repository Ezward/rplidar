"""
Stream lidar measurements to stdout as they are received.

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
    parser.add_argument("-n", "--number", type=int, default=1, help = "Number of scans to collect")
    parser.add_argument("-a", "--min_angle", type=float, default=0, help="Minimum angle in degress (inclusive) to save")
    parser.add_argument("-A", "--max_angle", type=float, default=360, help="Maximum angle in degrees (inclusive) to save")
    parser.add_argument("-d", "--min_distance", type=float, default=sys.float_info.min, help="Minimum distance (inclusive) to save")
    parser.add_argument("-D", "--max_distance", type=float, default=sys.float_info.max, help="Maximum distance (inclusive) to save")
     
    # Read arguments from command line
    args = parser.parse_args()
    
    help = []
    if args.number <= 0:
        help.append("-n/--number: must be positive.")
        
    if args.min_distance < 0:
        help.append("-d/--min_distance must be non-negative")

    if args.max_distance <= 0:
        help.append("-D/--max_distance must be positive")
        
    if args.min_angle < 0:
        help.append("-a/--min_angle must be non-negative")

    if args.max_angle <= 0:
        help.append("-A/--max_angle must be positive")
      
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
        
        lidar_info = lidar.info
                
        scan_count = 0
        last_angle = 0
        for new_scan, quality, angle, distance in lidar.iter_measurements():
            # check for start of new scan
            if new_scan:
                scan_count += 1
                
            if scan_count > args.number:
                break
                        
            if angle >= args.min_angle and angle <= args.max_angle:
                if distance >= args.min_distance and distance <= args.max_distance:
                    # convert to (x,y) point on plane
                    radians = angle * pi / 180.0
                    x = distance * cos(radians)
                    y = distance * sin(radians)
                    
                    measurement = {
                        "scan": scan_count,
                        "time": time.time(),
                        "distance": distance,
                        "angle": angle,
                        "x": x,
                        "y": y
                    }
                    print(json.dumps(measurement, indent=2))
            
            last_angle = angle
                    
    except KeyboardInterrupt:
        print('Stopping early.')
    except Exception as e:
        print(e)
        exit(1)
    finally:
        if lidar is not None:
            lidar.stop()
            lidar.disconnect()

