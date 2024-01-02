import json

import numpy as np

class Quaternion:
    def __init__(self):
        self.w = 0
        self.x = 0
        self.y = 0
        self.z = 1
    def define(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_euler(roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = Quaternion()

        q.define(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)

        return q

    def to_euler(self):
        roll = np.arctan2(2*(self.w*self.x+self.y*self.z), 1-2*(self.x**2+self.y**2))
        pitch = np.arcsin(2*(self.w*self.y-self.z*self.x))
        yaw = np.arctan2(2*(self.w*self.z+self.x*self.y), 1-2*(self.y**2+self.z**2))
        return roll, pitch, yaw

class Geometry:
    @staticmethod
    def on_segment(p, q, r):
        """
        Check if point q lies on line segment pr
        """
        return (max(p.x, r.x) >= q.x >= min(p.x, r.x) and
                max(p.y, r.y) >= q.y >= min(p.y, r.y))

    @staticmethod
    def line_intersects_line(line1_start, line1_end, line2_start, line2_end):
        def orientation(p, q, r):
            val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
            if val == 0:
                return 0  # Collinear points
            return 1 if val > 0 else 2  # Clockwise or Counterclockwise
        o1 = orientation(line1_start, line1_end, line2_start)
        o2 = orientation(line1_start, line1_end, line2_end)
        o3 = orientation(line2_start, line2_end, line1_start)
        o4 = orientation(line2_start, line2_end, line1_end)

        if o1 != o2 and o3 != o4:
            # Calculate intersection point
            intersect_x = (
                                  (line1_start.x * line1_end.y - line1_start.y * line1_end.x) * (
                                  line2_start.x - line2_end.x) -
                                  (line2_start.x * line2_end.y - line2_start.y * line2_end.x) * (
                                          line1_start.x - line1_end.x)
                          ) / (
                                  (line1_start.x - line1_end.x) * (line2_start.y - line2_end.y) -
                                  (line1_start.y - line1_end.y) * (line2_start.x - line2_end.x)
                          )
            intersect_y = (
                                  (line1_start.x * line1_end.y - line1_start.y * line1_end.x) * (
                                  line2_start.y - line2_end.y) -
                                  (line2_start.x * line2_end.y - line2_start.y * line2_end.x) * (
                                          line1_start.y - line1_end.y)
                          ) / (
                                  (line1_start.x - line1_end.x) * (line2_start.y - line2_end.y) -
                                  (line1_start.y - line1_end.y) * (line2_start.x - line2_end.x)
                          )
            intersection_point = Geometry.Point(intersect_x, intersect_y)

            # Check if intersection point lies within both line segments
            if (Geometry.on_segment(line1_start, intersection_point, line1_end) and
                    Geometry.on_segment(line2_start, intersection_point, line2_end)):
                return intersection_point
            else: return False

        return False

    @staticmethod
    def line_intersects_rect(line_start, line_end, rect_position, rect_size): #TODO: Return false if on segment

        def orientation(p, q, r):
            val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
            if val == 0:
                return 0  # Collinear points
            return 1 if val > 0 else 2  # Clockwise or Counterclockwise

        def do_segments_intersect(p1, q1, p2, q2):
            o1 = orientation(p1, q1, p2)
            o2 = orientation(p1, q1, q2)
            o3 = orientation(p2, q2, p1)
            o4 = orientation(p2, q2, q1)

            if o1 != o2 and o3 != o4:
                return True

            if o1 == 0 and Geometry.on_segment(p1, p2, q1):
                return True
            if o2 == 0 and Geometry.on_segment(p1, q2, q1):
                return True
            if o3 == 0 and Geometry.on_segment(p2, p1, q2):
                return True
            if o4 == 0 and Geometry.on_segment(p2, q1, q2):
                return True

            return False

        # Define the rectangle's four corners
        rect_end = Geometry.Point(rect_position.x + rect_size.width, rect_position.y + rect_size.height)
        top_left = rect_position
        top_right = Geometry.Point(rect_end.x, rect_position.y)
        bottom_left = Geometry.Point(rect_position.x, rect_end.y)
        bottom_right = rect_end
        if line_start.x == line_end.x and (line_start.x == top_right.x or line_start.x == top_left.x):
            return False
        if line_start.y == line_end.y and (line_start.y == top_right.y or line_start.y == bottom_right.y):
            return False
        # Check if the line segment intersects with any of the rectangle's edges
        if (do_segments_intersect(line_start, line_end, top_left, top_right) or
                do_segments_intersect(line_start, line_end, top_right, bottom_right) or
                do_segments_intersect(line_start, line_end, bottom_right, bottom_left) or
                do_segments_intersect(line_start, line_end, bottom_left, top_left)):
            return True

        return False

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

        def distance(self, point):
            return ((self.x - point.x) ** 2 + (self.y - point.y) ** 2) ** 0.5

        def unpack(self):
            return self.x, self.y

    class Size:
        def __init__(self, width, height):
            self.width = width
            self.height = height

        def unpack(self):
            return self.width, self.height

        def add(self, size):
            return Geometry.Size(self.width + size.width, self.height + size.height)

    class Obstacle:
        def __init__(self, location, size, rot):
            self.Loc = location
            self.Size = size
            self.Rot = rot

def json_to_obj(jsonfile):
    jsonfile = json.loads(jsonfile)
    try:
        if jsonfile["Type"] == "Map":
            from frc_map import Map
            mapreturn = Map(jsonfile["JSON"])
            return mapreturn
    except:
        if jsonfile[0] == "Map":
            from frc_map import Map
            mapreturn = Map(jsonfile[1])
            return mapreturn