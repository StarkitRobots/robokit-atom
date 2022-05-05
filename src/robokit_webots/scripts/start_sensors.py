import rospy
import rosservice
import message_filters as mf
from webots_ros.srv  import set_int
from webots_ros.msg  import Float64Stamped
from sensor_msgs.msg import JointState

ROBOT_NAME = "beep_beep"
SAMPLING_PERIOD = 32
JOINT_STATE_TOPIC = "/joint_state"


def psensor2joint(psensor):
    """Transforms position sensor name wit prefix to joint name"""
    return "_".join(psensor.split("/")[2].split("_")[:-1])


def get_sensors():
    """Gets lists of position sensors and IMU's by parsing the list of available services"""
    srv_list = rosservice.get_service_list()
    psensor_set = set()
    imu_set = set()

    for srv in srv_list:
        srv_name = srv.split("/")
        if srv_name[1] == ROBOT_NAME:

            if srv_name[2].split("_")[-1] == "sensor":
                psensor = "/".join(srv_name[:-1])
                psensor_set.add(psensor)    
            if srv_name[2].split("_")[0] == "imu":
                imu = "/".join(srv_name[:-1])
                imu_set.add(imu)

    ## HARDCODED
    psensor_set.remove("/" + ROBOT_NAME + "/battery_sensor")

    psensor_list = list(psensor_set)
    imu_list     = list(imu_set)

    return psensor_list, imu_list


def start_sensors(sensors):
    """Enables position sensors and IMU's"""
    psensor_list, imu_list = sensors
    for sensor in (psensor_list + imu_list):
        enable_sensor = rospy.ServiceProxy(sensor + "/enable", set_int)
        res = enable_sensor(SAMPLING_PERIOD)
        if not res.success:
            print("Error: failed to start sensor", sensor)


def joint_state_publisher(psensor_list):
    """Collects data from position sensors and publishes it in /joint_state topic"""
    joints = list(map(psensor2joint, psensor_list))

    subscribers = [mf.Subscriber(psensor + "/value", Float64Stamped) for psensor in psensor_list]
    js_publisher = rospy.Publisher(JOINT_STATE_TOPIC, JointState, queue_size = 10)

    ## Needs to be tweaked
    queue_size = 10
    fps = 100.
    delay = 1 / fps * 0.5

    def joints_callback(*psensors_msgs):
        joint_state = JointState()
        time = rospy.Time.now()

        joint_state.header.stamp.secs  = time.secs
        joint_state.header.stamp.nsecs = time.nsecs
        for msg, joint in zip(psensors_msgs, joints):
            joint_state.name.append(joint)
            joint_state.position.append(msg.data)
        js_publisher.publish(joint_state)

    mfilter = mf.ApproximateTimeSynchronizer(subscribers, queue_size, delay)
    mfilter.registerCallback(joints_callback)


if __name__ == "__main__":
    rospy.init_node('joint_state_publisher', anonymous=True)

    robot_sensors = get_sensors()
    start_sensors(robot_sensors)

    robot_psensor_list, _ = robot_sensors

    joint_state_publisher(robot_psensor_list)

    rospy.spin()
