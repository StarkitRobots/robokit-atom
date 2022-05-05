from math import sin

import rospy
from robokit_webots.msg import servo_command

STEP = 3.14 / 6

rospy.init_node('example_servo_publisher', anonymous=True)
pub = rospy.Publisher('servo_control', servo_command, queue_size=10)

msg = servo_command()
msg.names = ['head_yaw', 'right_shoulder_pitch', 'left_shoulder_pitch']

r = rospy.Rate(1)
x = 0

while not rospy.is_shutdown():
    msg.values = [sin(x), sin(x), sin(x)]
    pub.publish(msg)
    x += STEP
    r.sleep()
