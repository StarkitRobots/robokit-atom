# TODO rename script

from random import randint
from time import sleep

import rospy
from robokit_webots.msg import servo_command
from robokit_webots.srv import SetSource
import message_filters as mf


TOPICS = "walk_joint_goals", "kick_joint_goals", "animation_joint_goals", "head_joint_goals", "fake_joint_goals"
SERVO_CONTROL_TOPIC = "servo_control"
SOURCES = "walk_engine", "kick_engine", "animation_server", "head_control", "fake"

class Manager:

    def __init__(self):
        self._pub = rospy.Publisher(SERVO_CONTROL_TOPIC, servo_command, queue_size=10)
        self._current_source = "fake"
        for topic in TOPICS: rospy.Subscriber(topic, servo_command, self.joint_goals_filter_callback)
        rospy.Service('set_source', SetSource, self.handle_set_source)

    def joint_goals_filter_callback(self, joint_goal):
        if joint_goal.source in self._current_source:
            self._pub.publish(joint_goal)

    def handle_set_source(self, req):
        return self.set_source(req.source)

    def set_source(self, source):
        if source not in SOURCES:
            print(f"Error: Source {source} is not defined")
            return False

        print(f"Source switched from {self._current_source} to {source}")
        self._current_source = source
        return True

if __name__ == "__main__":
    rospy.init_node('manager', anonymous=True)
    manager = Manager()
    manager.set_source("animation_server")
    rospy.spin()
    #manager.test()