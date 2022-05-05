import os

import json
import rospy
from robokit_webots.msg import servo_command

ACTION_PATH = os.path.dirname(__file__) + "/../actions/"
SAMPLING_PERIOD = 32

## Maybe we should use path instead of action name
def load_action(action_name):
    "Loads action json file from ../action folder"
    try:
        json_file = open(ACTION_PATH + action_name, "r", encoding="utf-8")
    except FileNotFoundError:
        print(f"Error: Action '{action_name}' not found inside action folder")
        return None
    else:
        with json_file:
            action = json.loads(json_file.read())
            return action

def execute_action(servo_pub, action):
    "Executes desired action"
    joints = action["header"]["joints"]
    keypoints = action["keypoints"]

    msg = servo_command()
    msg.names = joints

    rate = rospy.Rate(1000 / SAMPLING_PERIOD)

    def send(values):
        print(f"Sent {values} to {joints}")
        msg.values = values
        servo_pub.publish(msg)

    def transfer_keypoint(keypoint, keypoint_next):
        frames = keypoint_next["frames"]

        slopes = []
        biases = []

        for pose_start, pose_end in zip(keypoint["values"], keypoint_next["values"]):
            slopes.append((pose_end - pose_start) / frames)
            biases.append(pose_start)

        for frame in range(frames):
            values = [bias + slope*frame for slope, bias in zip(slopes, biases)]
            send(values)
            rate.sleep()

    send(keypoints[0]["values"])

    for keypoint, keypoint_next in zip(keypoints[:-1], keypoints[1:]):
        val1 = keypoint["values"]
        val2 = keypoint_next["values"]
        frames = keypoint_next["frames"]

        print(f"Set keypoints {val1} -> {val2} for {joints} in {frames} frames")
        transfer_keypoint(keypoint, keypoint_next)
        print("---\n")

if __name__ == "__main__":
    rospy.init_node('executor', anonymous=False)
    test_action = load_action("test_action.json")
    pub = rospy.Publisher('servo_control', servo_command, queue_size=10, latch=True)
    execute_action(pub, test_action)
