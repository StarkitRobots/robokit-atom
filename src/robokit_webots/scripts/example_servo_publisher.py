from math import sin

import rospy
from robokit_webots.msg import servo_command

STEP = 3.14 / 6
FAKE_TOPIC = "fake_joint_goals"

rospy.init_node('example_servo_publisher', anonymous=True)
pub = rospy.Publisher(FAKE_TOPIC, servo_command, queue_size=10)

msg = servo_command()
msg.names = ['head_yaw', 'right_shoulder_pitch', 'left_shoulder_pitch']
msg.source = "fake"

r = rospy.Rate(1)
x = 0

while not rospy.is_shutdown():
    msg.values = [sin(x), sin(x), sin(x)]
    pub.publish(msg)
    x += STEP
    r.sleep()
