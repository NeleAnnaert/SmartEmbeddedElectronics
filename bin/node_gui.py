#!/usr/bin/env python
from mapper import Mapper, Location, GUI
from motor_control.msg import motor as MotorMsg
from aruco.msg import Position as Pos
from laser_detect.msg import Distance as Obj
from parking_detect.msg import Orientation as Park
import rospy


# The callback functions

def cb_pos(data):
    # Convert rospy data to pure Python type
    # Convert data to location class
    location = Location(data.X_coord, data.Y_coord, data.Robot_Rotation)
    mapper.update_pos(location)


def cb_obj(data):
    mapper.update_obj(data.distance)


def cb_park(data):
    mapper.update_park(data.rotation)


def send_motor(length, angles, distances):
    msg = MotorMsg()
    msg.length = length
    msg.array_angle = angles
    msg.array_distance = distances
    pub.publish(msg)


def motor_up(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(0)]
    msg.array_distance = [float(1)]
    pub.publish(msg)


def motor_down(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(0)]
    msg.array_distance = [float(-1)]
    pub.publish(msg)


def motor_left(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(90)]
    msg.array_distance = [float(1)]
    pub.publish(msg)


def motor_right(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(-90)]
    msg.array_distance = [float(1)]
    pub.publish(msg)


def motor_angle_left(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(5)]
    msg.array_distance = [float(0)]
    pub.publish(msg)


def motor_angle_right(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(-5)]
    msg.array_distance = [float(0)]
    pub.publish(msg)
    

if __name__ == "__main__":
    width = 159
    height = 96
    mapper = Mapper(width, height, send_motor)
    pub = rospy.Publisher("mapper", MotorMsg, queue_size=10)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("mapper_node")

    rospy.Subscriber("position", Pos, cb_pos)
    rospy.Subscriber("object", Obj, cb_obj)
    rospy.Subscriber("parking", Park, cb_park)

    app = GUI(mapper, motor_up=motor_up, motor_down=motor_down, motor_left=motor_left, motor_right=motor_right,
              motor_angle_left=motor_angle_left, motor_angle_right=motor_angle_right)
app.mainloop()
