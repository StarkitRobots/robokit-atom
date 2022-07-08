#!/usr/bin/env python3  
import sys
"""
pointgrey_camera_driver (at least the version installed with apt-get) doesn't
properly handle camera info in indigo.
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.

The yaml parsing is courtesy ROS-user Stephan:
    http://answers.ros.org/question/33929/camera-calibration-parser-in-python/

This file just extends that parser into a rosnode.
"""
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
import tf 

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data

    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "robokit/camera"
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def handle_transform(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")
if __name__ == "__main__":
    rospy.init_node("camera_info_publisher", anonymous=True)
    # Get fname from command line (cmd line input required)
    import argparse
    arg_parser = argparse.ArgumentParser()
    # Legacy!!!!!
    print(sys.argv)
    arg_parser.add_argument("filename", help="Path to yaml file containing " +\
                                             "camera calibration data")

    arg_parser.add_argument("a", help="Path to yaml file containing " +\
                                             "camera calibration data")
    arg_parser.add_argument("b", help="Path to yaml file containing " +\
                                             "camera calibration data")

    args = arg_parser.parse_args()
    filename = args.filename

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    publisher = rospy.Publisher("/robokit/camera/camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(3.14, 0, 3.14),
                     rospy.Time.now(),
                     "robokit/camera",
                     "camera")
        publisher.publish(camera_info_msg)
        rate.sleep()    
