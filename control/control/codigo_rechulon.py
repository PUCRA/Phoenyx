import math
from sensor_msgs.msg import LaserScan
import numpy as np

def rotate_laserscan(scan_msg: LaserScan, angle_shift_rad: float) -> LaserScan:
    ranges = np.array(scan_msg.ranges)
    angle_increment = scan_msg.angle_increment
    shift = int(angle_shift_rad / angle_increment)

    # Rota el array circularmente
    rotated_ranges = np.roll(ranges, shift)

    # Crear nuevo mensaje corregido
    corrected_scan = LaserScan()
    corrected_scan.header = scan_msg.header
    corrected_scan.header.frame_id = 'base_link'  # o lo que quieras
    corrected_scan.angle_min = scan_msg.angle_min
    corrected_scan.angle_max = scan_msg.angle_max
    corrected_scan.angle_increment = scan_msg.angle_increment
    corrected_scan.time_increment = scan_msg.time_increment
    corrected_scan.scan_time = scan_msg.scan_time
    corrected_scan.range_min = scan_msg.range_min
    corrected_scan.range_max = scan_msg.range_max
    corrected_scan.ranges = rotated_ranges.tolist()
    corrected_scan.intensities = scan_msg.intensities

    return corrected_scan
