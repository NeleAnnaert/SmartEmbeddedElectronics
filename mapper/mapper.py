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


def parking_vectors(height, width):
    """
    Generates the wall vectors based on an height and width
    :param height: int of the height
    :param width: int of the width
    :return: An list with vectors
    """
    width_pos = int(width / 2)
    width_neg = -width_pos
    height_pos = int(height / 2)
    height_neg = -height_pos
    vec = []
    for x in range(width_neg, width_pos):
        vec.append((height_pos, x))
        vec.append((height_neg, x))
    for y in range(height_neg + 1, height_pos):
        vec.append((y, width_neg))
    return vec


class Mapper:
    """
    The class that holds the map. It has function for the callback functions and sends an messages to the motor node.
    This messages contains the steps needed to take to reach the parking place.
    Static:
        OFFSET_POS: the max distance that the robot goes off the pat
        OFFSET_PARK: the max distance that the robot can deviate from the parking spot
        DIST_PARK: parameter to look if we are close to the parking spot or not
        WIDTH_PARK: The width of the parking
        LENGTH_PARK: The length of the parking
        PARKING_WALL: An number that represents the wall of the parking
        PARKING_VECTOR: An vector with the walls of the parking
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
        parking_found()
        path_plan()
        update()
        update_pos()
        update_obj()
        update_park()
        place_map()
        place_wall()
        add_park()
        del_park()
        __str__()
        __repr__()
    """
    OFFSET_POS = 2
    OFFSET_PARK = 10
    DIST_PARK = 1
    WIDTH_PARK = 6
    LENGTH_PARK = 15
    PARKING_WALL = 1
    PARKING_VECTOR = parking_vectors(20, 18)
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
        self.changes_obj = False
        self.changes_parking = False

        map_center_x = int(self.width / 2)
        map_center_y = int(self.height / 2)
        self.current_loc = Location(map_center_x, map_center_y, 0)
        self.previous_loc = Location(map_center_x, map_center_y, 0)
        self.parking_loc = Location(map_center_x, map_center_y, 0)
        self.previous_park = Location(map_center_x, map_center_y, 0)

        self.path_found = None

        self._observers = []

        self.delete_obj()
        self.add_park()

    def changes(self):
        """
        Checks if there are any changes to the map, like if the robot has a new position, a new object has been found,
        the parking has been changed or found and if there is a new path.
        """
        if self.changes_loc or self.changes_obj or self.changes_parking:
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

    def obstacle_on_path(self):
        if self.changes_loc or self.changes_parking:
            return False
        else:
            if self.path_found and self.changes_obj:
                for pair in self.path_found:
                    x = pair[0]
                    y = pair[1]
                    if self.map_matrix[y][x] == self.PARKING_WALL or 0 < self.map_matrix[y][x]:
                        return True
        return False

    def parking_found(self):
        """
        Checks if the robot is in the parking at the right place with a little offset
        """
        for i in range(-self.OFFSET_PARK, self.OFFSET_PARK):
            x = self.current_loc.x + i
            for j in range(-self.OFFSET_PARK, self.OFFSET_PARK):
                y = self.current_loc.y + j
                if self.current_loc == self.parking_loc:
                    return True
        return False

    def path_plan(self):
        """
        Finds the path and stores this in the attribute self.path_found and also sends an message to the motor.
            self.path_found contains an list of pairs (x,y) that are the coordinates of the path
            self.send_motor(args) Sends an message to the motor
        """
        path = Path(matrix=self.map_matrix, start_x=self.current_loc.x, start_y=self.current_loc.y,
                    end_x=self.parking_loc.x, end_y=self.parking_loc.y, angle_start=self.current_loc.angle,
                    angle_end=self.parking_loc.angle, weight=self.weight)
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
        the parking. If not we will plan an path and send an message to the motor.
        """
        if self.changes():
            if self.obstacle_on_path() or not self.on_path() or not self.parking_found():
                self.path_plan()
            self.changes_loc = False
            self.changes_obj = False
            self.changes_parking = False

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

    def delete_obj(self):
        threading.Timer(self.UPDATE_MAP_TIME, self.delete_obj).start()
        current_time = int(time.time())
        current_time_del = current_time - self.DELETE_TIME  # The time x second ago
        change = False
        for i in range(self.height):
            for j in range(self.width):
                val = self.map_matrix[i][j]
                if (0 < val < current_time_del) and (val != self.PARKING_WALL):
                    self.map_matrix[i][j] = 0
                    change = True
        if change:
            for cb in self._observers:
                cb()

    def update_obj(self, list_dist):
        """
        Updates the objects on the map that are detected by the laser.
        If the objects on the map are older then 10 seconds we delete them to have a more dynamic map.
        """
        print(list_dist)
        current_time = int(time.time())
        # The length of the list
        length_list = len(list_dist) - 1

        # Points density in function of the angle
        points_density = self.VIEWING_ANGLE / length_list
        angle_points_density = 0
        for i in range(length_list, -1, -1):
            if list_dist[i] == -1:
                continue  # No object detected so pass
            else:
                distance = float(list_dist[i])
                # Map list element to an map coordinate
                angle_offset = angle_points_density - self.VIEWING_ANGLE_D2
                angle_point = self.current_loc.angle + angle_offset
                radian_point = radians(angle_point)
                x_dist = cos(radian_point) * distance
                y_dist = sin(radian_point) * distance
                x = self.current_loc.x + int(x_dist)
                y = self.current_loc.y - int(y_dist)
                try:
                    if self.map_matrix[y][x] != self.PARKING_WALL \
                            and x != self.current_loc.x and y != self.current_loc.y:
                        x_0 = x - 1
                        x_2 = x + 1
                        y_0 = y - 1
                        y_2 = y + 1

                        self.place_map(y_0, x_0, current_time)
                        self.place_map(y_0, x, current_time)
                        self.place_map(y_0, x_2, current_time)

                        self.place_map(y, x_0, current_time)
                        self.place_map(y, x, current_time)
                        self.place_map(y, x_2, current_time)

                        self.place_map(y_2, x_0, current_time)
                        self.place_map(y_2, x, current_time)
                        self.place_map(y_2, x_2, current_time)
                except IndexError:
                    pass

            angle_points_density += points_density
        self.changes_obj = True
        for cb in self._observers:
            cb()

    def update_park(self, angle):
        """
        Updates the angle of the parking spot and calls the function add_park to build a wall around the parking spot.
        """
        angle = int(angle)
        self.previous_park.angle = self.parking_loc.angle
        self.parking_loc.angle = self.current_loc.angle + angle - 180
        if sqrt((self.current_loc.x - self.parking_loc.x) ** 2 + (self.current_loc.y - self.parking_loc.y) ** 2) \
                <= self.DIST_PARK:
            if self.parking_loc != self.previous_park:
                self.changes_parking = True
            else:
                self.changes_parking = False
            self.del_park()
            self.add_park()
        else:
            self.changes_parking = False
            # Only delete parking walls when we are setting new walls
        for cb in self._observers:
            cb()

    def place_map(self, x, y, item):
        """ Place item on map for given coordinates"""
        try:
            self.map_matrix[x][y] = item
        except IndexError:
            pass

    def place_wall(self, x, y):
        """ Place wall on map for given coordinates"""
        return self.place_map(x, y, self.PARKING_WALL)

    def add_park(self):
        """
        Calculates and builds a wall around the parking spot. To make sure we don't drive over the lines of the parking
        spot.
        """
        vec = rotate_vectors(self.PARKING_VECTOR, -self.parking_loc.angle)
        for cor in vec:
            x = int(cor[0]) + self.parking_loc.x
            y = int(cor[1]) + self.parking_loc.y

            x_0 = x - 1
            x_2 = x + 1
            y_0 = y - 1
            y_2 = y + 1

            self.place_wall(y_0, x_0)
            self.place_wall(y_0, x)
            self.place_wall(y_0, x_2)

            self.place_wall(y, x_0)
            self.place_wall(y, x)
            self.place_wall(y, x_2)

            self.place_wall(y_2, x_0)
            self.place_wall(y_2, x)
            self.place_wall(y_2, x_2)

    def del_park(self):
        """
        If the parking spot has been updated we call this function to delete the previous parking spot
        """
        for i in range(self.height):
            for j in range(self.width):
                if self.map_matrix[i][j] == self.PARKING_WALL:
                    self.map_matrix[i][j] = 0

    def __str__(self):
        string = "Mapper\n"
        string += "__________________________________________________________________________\n"
        string += "\twidth = " + str(self.width) + "\n"
        string += "\theight = " + str(self.height) + "\n"
        string += "\tMap matrix\n"
        string += repr(self)
        string += "\tchanges_loc = " + str(self.changes_loc) + "\n"
        string += "\tchanges_obj = " + str(self.changes_obj) + "\n"
        string += "\tchanges_parking =" + str(self.changes_parking) + "\n"
        string += "\tcurrent_location" + str(self.current_loc) + "\n"
        string += "\tprevious_location" + str(self.previous_loc) + "\n"
        string += "\tcurrent_parking" + str(self.parking_loc) + "\n"
        string += "\tprevious_parking" + str(self.previous_park) + "\n"
        string += "\n"
        return string

    def __repr__(self):
        string = "c: current location, p: previous location, e: parking location,\n"
        string += "q: previous parking location, x: path, ▒: parking wall and █: object\n  "
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
                elif self.parking_loc.x == col and self.parking_loc.y == row:
                    string += "e"
                elif self.previous_park.x == col and self.previous_park.y == row:
                    string += "q"
                elif self.path_found and ((col, row) in self.path_found):
                    string += "x"
                elif val == self.PARKING_WALL:
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
    # mapper.parking_loc = Location(25, 25, 85)
    loc_robot = Location(0, 0, 0)
    loc_park = Location(50, 50, -135)
    mapper.parking_loc = loc_park
    mapper.add_park()
    # mapper.changes_parking = True
    # mapper.update()
    # mapper.update_pos(loc)
    # print(repr(mapper))
    # loc = Location(25, 15, 0)
    # mapper.update_pos(loc)      #updating current_loc works!!!
    # print(repr(mapper))
    # mapper.update_park(-45)     #building walls around parkings works!!!
    # print(repr(mapper))
    mapper.update_pos(loc_robot)
    # mapper.update_obj([10, 12, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 5])

    mapper.path_plan()
    # print(mapper.current_loc)
print(repr(mapper))
