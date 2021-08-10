"""
Consume LIDAR measurement file and produce sorted output file.

Adafruit invests time and resources providing this open source code.
Please support Adafruit and open source hardware by purchasing
products from Adafruit!

Written by Dave Astels for Adafruit Industries
Copyright (c) 2019 Adafruit Industries
Licensed under the MIT license.

All text above must be included in any redistribution.

Modified by Ezward for filtering and console logging of lidar data

insort_right() and bisect_right() copied from cpython
under this license https://github.com/python/cpython/blob/main/LICENSE
"""

import argparse
import json
from math import cos, sin, pi, floor
import sys

from adafruit_rplidar import RPLidar

def insort_right(a, x, lo=0, hi=None, *, key=None):
    """Insert item x in list a, and keep it sorted assuming a is sorted.

    If x is already in a, insert it to the right of the rightmost x.

    Optional args lo (default 0) and hi (default len(a)) bound the
    slice of a to be searched.
    """
    if key is None:
        lo = bisect_right(a, x, lo, hi)
    else:
        lo = bisect_right(a, key(x), lo, hi, key=key)
    a.insert(lo, x)

def bisect_right(a, x, lo=0, hi=None, *, key=None):
    """Return the index where to insert item x in list a, assuming a is sorted.

    The return value i is such that all e in a[:i] have e <= x, and all e in
    a[i:] have e > x.  So if x already appears in the list, a.insert(i, x) will
    insert just after the rightmost x already there.

    Optional args lo (default 0) and hi (default len(a)) bound the
    slice of a to be searched.
    """

    if lo < 0:
        raise ValueError('lo must be non-negative')
    if hi is None:
        hi = len(a)
    # Note, the comparison uses "<" to match the
    # __lt__() logic in list.sort() and in heapq.
    if key is None:
        while lo < hi:
            mid = (lo + hi) // 2
            if x < a[mid]:
                hi = mid
            else:
                lo = mid + 1
    else:
        while lo < hi:
            mid = (lo + hi) // 2
            if x < key(a[mid]):
                hi = mid
            else:
                lo = mid + 1
    return lo



def filter_lidar_scan(scan, min_angle = 0, max_angle = 360, min_distance = sys.float_info.min, max_distance = sys.float_info.max, sorted=False):
    nearest_distance = None
    nearest_angle = None
    nearest_x = None
    nearest_y = None

    farthest_distance = None
    farthest_angle = None
    farthest_x = None
    farthest_y = None
    
    filtered_data = []

    for (_, angle, distance) in scan:
        if angle >= min_angle and angle <= max_angle:
            if distance >= min_distance and distance <= max_distance:
                radians = angle * pi / 180.0
                x = distance * cos(radians)
                y = distance * sin(radians)
                
                measurement = {
                    "distance": distance,
                    "angle": angle,
                    "x": x,
                    "y": y
                }
                
                if sorted:
                    insort_right(filtered_data, measurement, key=lambda m: m["angle"])
                else:
                    filtered_data.append(measurement)

                if farthest_distance is None or distance > farthest_distance:
                    farthest_distance = distance
                    farthest_angle = angle
                    farthest_x = x
                    farthest_y = y
                
                if nearest_distance is None or distance < nearest_distance:
                    nearest_distance = distance
                    nearest_angle = angle
                    nearest_x = x
                    nearest_y = y
    
    return {
        "scan": filtered_data,
        "far": {
            "distance": farthest_distance,
            "angle": farthest_angle,
            "x": farthest_x,
            "y": farthest_y
        },
        "near": {
            "distance": nearest_distance,
            "angle": nearest_angle,
            "x": nearest_x,
            "y": nearest_y
        },
    }
                    

def dump_lidar_data(data):
    print(json.dumps(data, indent=2))
    
if __name__ == "__main__":
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--number", type=int, default=1, help = "Number of scans to collect")
    parser.add_argument("-s", "--sort", action='store_true', help="Sort the measurements by angle")
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
        
        count = 0
        for scan in lidar.iter_scans():
            filtered = filter_lidar_scan(scan,
                                         min_angle=args.min_angle,
                                         max_angle=args.max_angle,
                                         min_distance=args.min_distance,
                                         max_distance=args.max_distance,
                                         sorted=args.sort)
            lidar_scans.append(filtered)
            count += 1
            if count >= args.number:
                break

    except KeyboardInterrupt:
        print('Stopping early.')
    except Exception as e:
        print(e)
        exit(1)
    finally:
        if lidar is not None:
            lidar.stop()
            lidar.disconnect()

    lidar_data = {
        "info": lidar_info,
        "scans": lidar_scans
    }
    print(json.dumps(lidar_data, indent=2))
