# -*- coding: utf-8 -*-
# !/usr/bin/env python
import threading
from math import radians, cos, sin, tan, sqrt
from path_finder import Path
import time


class Location:
    """
    An class that holds information over an location:
    Attributes:
        x: The x coordinate
        y: The y coordinate
        angle: The angle
    Functions:
        __init__(x, y, angle)
        __eq__(other)
        __str__()
        __repr__()
    """

    def __init__(self, x, y, angle):
        self.x = int(round(x, 0))
        self.y = int(round(y, 0))
        self.angle = int(round(angle, 0))

    def __eq__(self, other):
        try:
            if self.x == other.x and self.y == other.y and self.angle == other.angle:
                return True
            else:
                return False
        except AttributeError:
            return False

    def __str__(self):
        string = "Locatie\n"
        string += "\tx = " + str(self.x) + "\n"
        string += "\ty = " + str(self.y) + "\n"
        string += "\ta = " + str(self.angle) + "\n"
        return string

    def __repr__(self):
        return "%i %i %i" % (self.x, self.y, self.angle)


def rotate_vectors(vectors, angle):
    """Rotates the given vectors which consists of corners represented as (x,y),
    around the ORIGIN, counter clock-wise, theta degrees"""
    theta = radians(angle - 90)
    rot_vec = []
    for corner in vectors:
        rot_vec.append((corner[0] * cos(theta) - corner[1] * sin(theta),
                        corner[0] * sin(theta) + corner[1] * cos(theta)))
    return rot_vec


class Mapper:
    """
    The class that holds the map. It has function for the callback functions and sends an messages to the motor node.
    This messages contains the steps needed to take to reach the parking place.
    Static:
        OFFSET_POS: the max distance that the robot goes off the pat
        DELETE_TIME: The time in ms before we delete an object
        VIEWING_ANGLE: The viewing angle of the camera.
        VIEWING_ANGLE_D2: The viewing angle of the camera divide by 2.
    Attributes:
        width: The width of the map (x).
        height: The height of the map (y).
        send_motor: Function that would be called for sending an messages to an motor.
                    Will be called so: send_motor(length, angles, distances)
        map_matrix: The map an list in an list.
        current_loc: The current location an element of class Location.
        parking_loc: The location of the parking spot.
        path_found: An list of tuples of the founded path.
    Functions:
        __init__(width, height, send_motor)
        changes()
        on_path()
        path_plan()
        update()
        update_pos()
        update_obj()
        place_map()
        place_wall()
        add_park()
        del_park()
        __str__()
        __repr__()
    """
    OFFSET_POS = 2
    PARKING_WALL = 1
    DELETE_TIME = 60  # 60 seconds
    UPDATE_MAP_TIME = 120  # 120 seconds

    # First define the viewing angle of the camera this is an constant.
    VIEWING_ANGLE = 90
    VIEWING_ANGLE_D2 = VIEWING_ANGLE / 2

    def __init__(self, width, height, send_motor):
        self.send_command = False
        self.weight = -2
        self.width = width
        self.height = height
        self.send_motor = send_motor

        self.map_matrix = []
        for i in range(self.height):
            self.map_matrix.append([0] * self.width)

        self.changes_loc = False

        map_center_x = int(self.width / 2)
        map_center_y = int(self.height / 2)
        self.current_loc = Location(map_center_x, map_center_y, 0)
        self.previous_loc = Location(map_center_x, map_center_y, 0)

        self.path_found = None

        self._observers = []

    def changes(self):
        """
        Checks if there are any changes to the map, like if the robot has a new position, a new object has been found,
        """
        if self.changes_loc: 
            return True
        else:
            return False

    def on_path(self):
        """
        Checks if the robot stays on the path with a little offset
        """
        if self.path_found:
            for i in range(-self.OFFSET_POS, self.OFFSET_POS):
                x = self.current_loc.x + i
                for j in range(-self.OFFSET_POS, self.OFFSET_POS):
                    y = self.current_loc.y + j
                    pair = (x, y)
                    if pair in self.path_found:
                        return True
        return False

    def path_plan(self):
        """
        Finds the path and stores this in the attribute self.path_found and also sends an message to the motor.
            self.path_found contains an list of pairs (x,y) that are the coordinates of the path
            self.send_motor(args) Sends an message to the motor
        """
        path = Path(matrix=self.map_matrix, start_x=self.current_loc.x, start_y=self.current_loc.y,
        self.path_found = path.path
        length = len(path.waypoints)
        angles = []
        distances = []
        for waypoint in path.waypoints:
            angles.append(-waypoint.angle)
            distances.append(waypoint.length)
        if self.send_command:
            self.send_motor(length, angles, distances)
        for cb in self._observers:
            cb()

    def update(self):
        """
        Checks if there are any changes, when this is true it will check if the robot is still on path and is in't in
        """
        if self.changes():
            if not self.on_path(): 
                self.path_plan()
            self.changes_loc = False

    def update_pos(self, location):
        """
        Updates the position of the robot.
        """
        print(location)
        if (0 <= location.x < self.width) and (0 <= location.y < self.height) and (-360 <= location.angle <= 360):
            self.previous_loc = self.current_loc
            self.current_loc = location
            if self.current_loc != self.previous_loc:
                self.changes_loc = True
            else:
                self.changes_loc = False
            for cb in self._observers:
                cb()

    def place_map(self, x, y, item):
        """ Place item on map for given coordinates"""
        try:
            self.map_matrix[x][y] = item
        except IndexError:
            pass

    def __repr__(self):
        string = "c: current location, p: previous location, e: parking location,\n"
        row = 0
        for i in range(len(self.map_matrix[0])):
            string += "%3d " % i
        string += "\n"
        row = 0
        for row_list in self.map_matrix:
            col = 0
            string += "%3d " % row
            for val in row_list:
                if self.current_loc.x == col and self.current_loc.y == row:
                    string += "c"
                elif self.previous_loc.x == col and self.previous_loc.y == row:
                    string += "p"
                elif self.path_found and ((col, row) in self.path_found):
                    string += "x"
                    string += "▒"
                elif val != 0:
                    string += "█"
                else:
                    string += "_"
                string += "   "
                col += 1
            string += "\n"
            row += 1
        return string

    def add_observer(self, cb):
        self._observers.append(cb)


if __name__ == "__main__":
    # Test functions
    def _send_motor(*args, **kwargs):
        print("send motor")
        print("args")
        for count, thing in enumerate(args):
            print('{0}. {1}'.format(count, thing))
        print("kwargs")
        for name, value in kwargs.items():
            print('{0} = {1}'.format(name, value))


    _width = 159
    _height = 96
    mapper = Mapper(_width, _height, _send_motor)
    mapper.del_park()
    mapper.send_command = True
    loc_robot = Location(0, 0, 0)
    mapper.update_pos(loc_robot)
    # mapper.update_obj([10, 12, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 5])

    mapper.path_plan()
    # print(mapper.current_loc)
print(repr(mapper))
