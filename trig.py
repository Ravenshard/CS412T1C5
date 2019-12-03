import math
from geometry_msgs.msg import Point


def get_distance(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    return math.sqrt(dx**2 + dy**2)


def minimum_angle_between_headings(a, b):
    heading_difference = a - b
    if heading_difference < 0:
        heading_difference += 360
    if heading_difference > 180:
        heading_difference = b - a
        if heading_difference < 0:
            heading_difference += 360
        heading_difference = -heading_difference
    return heading_difference


def get_heading_between_points(start, end):
    dx = start.x - end.x
    dy = start.y - end.y
    heading = math.degrees(math.atan2(dy, dx))
    return heading+180


def get_point(point, distance, heading):
    calculated_point = Point()
    angle = math.radians(heading-180)
    calculated_point.x = point.x + distance * math.cos(angle)
    calculated_point.y = point.y + distance * math.sin(angle)
    calculated_point.z = point.z
    return calculated_point
