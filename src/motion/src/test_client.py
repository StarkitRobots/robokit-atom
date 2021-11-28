import rospy
from motion.srv import Motion, Walk

def walk_client(n, step, side, ang):
    rospy.wait_for_service('walk_service')
    try:
        a = rospy.ServiceProxy('walk_service', Walk)
        a(n, step, side, ang)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def motion_client(motion_id):
    rospy.wait_for_service('motion_service')
    try:
        a = rospy.ServiceProxy('motion_service', Motion)
        a(motion_id)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    walk_client(0, 0, 0, 0.0)
    motion_client('move_head')

        