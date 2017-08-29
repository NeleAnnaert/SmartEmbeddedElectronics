#!/usr/bin/env python
from mapper import Location, GUIStandalone
from mapper_node.msg import mapper_gui as MapperMsg
from motor_control.msg import motor as MotorMsg
import rospy, os, pickle
from std_srvs.srv import Trigger

try:
    import simplejson as json
except ImportError:
    import json


# The callback functions

def cb_mapper(data):
    if not os.path.isfile(mapper_lock_file):
        fp_cb = open(mapper_lock_file, "a")
        fp_cb.write("lock")
        fp_cb.close()
        fp_cb = open(mapper_file, "ab")
        pickle.dump(data, fp_cb)
        fp_cb.close()
        os.remove(mapper_lock_file)

def send_mapper(mapper):
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
    pub.publish(msg)


def motor_up(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(0)]
    msg.array_distance = [float(1)]
    pub_mot.publish(msg)


def motor_down(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(0)]
    msg.array_distance = [float(-1)]
    pub_mot.publish(msg)


def motor_left(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(90)]
    msg.array_distance = [float(1)]
    pub_mot.publish(msg)


def motor_right(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(-90)]
    msg.array_distance = [float(1)]
    pub_mot.publish(msg)


def motor_angle_left(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(5)]
    msg.array_distance = [float(0)]
    pub_mot.publish(msg)


def motor_angle_right(*args, **kwargs):
    msg = MotorMsg()
    msg.length = int(1)
    msg.array_angle = [float(-5)]
    msg.array_distance = [float(0)]
    pub_mot.publish(msg)

def button_3(*args, **kwargs):
    rospy.wait_for_service('position_calibration')  # The service name
    try:
        service = rospy.ServiceProxy('position_calibration', Trigger)  # Create service function
        response = service()  # Run service function
        return response  # Return the response
    except rospy.ServiceException:
        pass


app = None
if __name__ == "__main__":
    mapper_lock_file = "mapper.lock"
    mapper_file = "mapper.dat"
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("mapper_node_gui")

    pub = rospy.Publisher("mapper_gui_host", MapperMsg, queue_size=10)
    pub_mot = rospy.Publisher("mapper_gui_motor", MotorMsg, queue_size=10)
    rospy.Subscriber("mapper_gui", MapperMsg, cb_mapper)

    app = GUIStandalone(send_mapper, motor_up=motor_up, motor_down=motor_down, motor_left=motor_left,
                        motor_right=motor_right, motor_angle_left=motor_angle_left, motor_angle_right=motor_angle_right,
                        button_1=button_1, button_2=button_2, button_3=button_3)
    while True:
        if not os.path.isfile(mapper_lock_file):
            if os.path.isfile(mapper_file):
                fp = open(mapper_lock_file, "a")
                fp.write("lock")
                fp.close()

                fp = open(mapper_file, "rb")
                data = pickle.load(fp)
                fp.close()
                map_matrix = json.loads(data.map_matrix)
                current_loc = Location(data.current_loc_x, data.current_loc_y, data.current_loc_angle)
                previous_loc = Location(data.previous_loc_x, data.previous_loc_y, data.previous_loc_angle)
                weight = data.weight
                send_command = data.send_command
                app.update_mapper(map_matrix, current_loc, previous_loc, weight,
                                  send_command)
                os.remove(mapper_file)
                os.remove(mapper_lock_file)
        app.update_idletasks()
app.update()
