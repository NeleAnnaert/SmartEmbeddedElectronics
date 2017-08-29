#!/usr/bin/env/ python
from mapper import Mapper, Location
from motor_control.msg import motor as MotorMsg
from aruco.msg import Position as Pos
from mapper_node.msg import mapper_gui as MapperMsg
import rospy

try:
    import simplejson as json
except ImportError:
    import json


# The callback functions

def cb_pos(data):
    # Convert rospy data to pure Python type
    # Convert data to location class
    location = Location(data.X_coord, data.Y_coord, -data.Robot_Rotation)
    mapper.update_pos(location)

def send_motor(length, angles, distances):
    msg = MotorMsg()
    msg.length = length
    msg.array_angle = angles
    msg.array_distance = distances
    pub.publish(msg)


def cb_mapper(data):
    mapper.map_matrix = json.loads(data.map_matrix)
    mapper.current_loc = Location(data.current_loc_x, data.current_loc_y, data.current_loc_angle)
    mapper.previous_loc = Location(data.previous_loc_x, data.previous_loc_y, data.previous_loc_angle)
    mapper.weight = data.weight
    mapper.send_command = data.send_command
    mapper.path_plan()


def send_mapper():
    msg = MapperMsg()
    msg.map_matrix = json.dumps(mapper.map_matrix)
    msg.current_loc_x = mapper.current_loc.x
    msg.current_loc_y = mapper.current_loc.y
    msg.current_loc_angle = mapper.current_loc.angle
    msg.previous_loc_x = mapper.previous_loc.x
    msg.previous_loc_y = mapper.previous_loc.y
    msg.previous_loc_angle = mapper.previous_loc.angle
    msg.weight = mapper.weight
    msg.send_command = mapper.send_command
    pub_gui.publish(msg)


mapper = None
if __name__ == "__main__":
    width = 159
    height = 96
    mapper = Mapper(width, height, send_motor)
    mapper.add_observer(send_mapper)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("mapper_node")
    pub = rospy.Publisher("mapper", MotorMsg, queue_size=10)
    pub_gui = rospy.Publisher("mapper_gui", MapperMsg, queue_size=10)

    rospy.Subscriber("position", Pos, cb_pos)
    rospy.Subscriber("mapper_gui_host", MapperMsg, cb_mapper)

    # spin() simply keeps python from exiting until this node is stopped
rospy.spin()
