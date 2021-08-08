"""
Consume LIDAR measurement file and create an image for display.

Adafruit invests time and resources providing this open source code.
Please support Adafruit and open source hardware by purchasing
products from Adafruit!

Written by Dave Astels for Adafruit Industries
Copyright (c) 2019 Adafruit Industries
Licensed under the MIT license.

All text above must be included in any redistribution.

Modified by Ezward for filtering and console logging of lidar data
"""

import json
from math import cos, sin, pi, floor
import sys

from adafruit_rplidar import RPLidar

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

def filter_lidar_data(data, min_angle = 0, max_angle = 360, min_distance = sys.float_info.min, max_distance = sys.float_info.max):
    nearest_distance = None
    nearest_angle = None
    nearest_x = None
    nearest_y = None

    farthest_distance = None
    farthest_angle = None
    farthest_x = None
    farthest_y = None
    
    filtered_data = []

    for angle in range(360):
        if angle >= min_angle and angle <= max_angle:
            distance = data[angle]
            if distance >= min_distance and distance <= max_distance:
                radians = angle * pi / 180.0
                x = distance * cos(radians)
                y = distance * sin(radians)
                
                filtered_data.append({
                    "distance": distance,
                    "angle": angle,
                    "x": x,
                    "y": y
                })

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
        "data": filtered_data,
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
    

scan_data = [0]*360

try:
    print(lidar.info)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        processed_data = filter_lidar_data(scan_data)
        dump_lidar_data(processed_data)

except KeyboardInterrupt:
    print('Stoping.')
lidar.stop()
lidar.disconnect()
