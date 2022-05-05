#!/usr/bin/env python

import threading
from time import time

import rospy
import rosservice

from webots_ros.srv   import set_float
from robokit_webots.msg import servo_command


ROBOT_NAME = "beep_beep"
SERVO_CONTROL_TOPIC = "servo_control"
#robot_name = rospy.wait_for_message('/model_name', String).data


def get_srv_list():
    """Gets active services list"""
    srv_list = rosservice.get_service_list()
    return srv_list


def servo2srv(servo):
    """Transforms servo name to the corresponding /set_position service"""
    return '/' + ROBOT_NAME + '/' + servo + '/set_position'


# servo name set should be created manually if it is possible to avoid wrong entity calls
def get_robot_servo_set(srv_list):
    """Gets set of servos by parsing the list of available services"""
    new_servo_set = set()

    for srv in srv_list:
        srv = srv.split("/")
        if srv[1] == ROBOT_NAME:
            servo = srv[2]
            new_servo_set.add(servo)

    new_servo_set = clean_servo_set(new_servo_set)
    return new_servo_set


def clean_servo_set(raw_servo_set):
    """Cleans parsed servo set from names which correspond to unavailable services"""
    cleaned_servo_set = set()
    srv_list = get_srv_list()
    for servo in raw_servo_set:
        srv_name = servo2srv(servo)
        if srv_name in srv_list:
            cleaned_servo_set.add(servo)
    return cleaned_servo_set


def register_servo_dict(new_servo_set = None):
    """Returns a dict with name - set_position proxy pairs"""
    new_proxy_dict = {}

    if new_servo_set is None:
        srv_list = get_srv_list()
        new_servo_set = get_robot_servo_set(srv_list)

    for servo in new_servo_set:
        srv_name = servo2srv(servo)
        if srv_name in srv_list:
            client = rospy.ServiceProxy(srv_name, set_float)
            new_proxy_dict[servo] = client
        else:
            print(f"Trying to register proxy of service {srv_name}, which is not available")

    return new_proxy_dict, new_servo_set


def send_positions(servo_dict, servo_list, val_list):
    """Sends position commands to servos"""
    assert len(servo_list) == len(val_list)

    def send_command(servo, val):
        srv_name = servo2srv(servo)
        rospy.wait_for_service(srv_name)
        servo_dict[servo](val)

    for servo, val in zip(servo_list, val_list):
        if servo not in servo_dict:
            print(f"Wrong command: servo {servo} is not available")
            continue
        proxy_thread = threading.Thread(target=send_command, args=(servo, val))
        proxy_thread.start()


if __name__ == "__main__":
    rospy.init_node('servo_control', anonymous=True)

    robot_servo_dict, robot_servo_set = register_servo_dict()

    def servo_command_callback(msg):
        """Recieves servo commands and executes them"""
        print("Recieved: ", msg.names, msg.values)
        start = time()
        send_positions(robot_servo_dict, msg.names, msg.values)
        print(f"Processed in {(time() - start) * 1000} ms\n")

    rospy.Subscriber(SERVO_CONTROL_TOPIC, servo_command, servo_command_callback)
    print("Listening to commands...")
    rospy.spin()
