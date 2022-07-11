from email import header
from sel_map_utils import CarlaDataLoader

import numpy as np
import rospy
import sys
import os
import yaml
from scipy.spatial.transform import Rotation as R

import tf2_ros
import cv_bridge
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo


# Add static frames to the given broadcaster
def AddStaticFrame(staticBroadcaster, frame_id, frame_data=None):
    msg = TransformStamped()
    if frame_data:
        msg.child_frame_id = frame_id
        msg.header.frame_id = frame_data['parent']
        msg.transform.translation.x = frame_data['translation']['x']
        msg.transform.translation.y = frame_data['translation']['y']
        msg.transform.translation.z = frame_data['translation']['z']
        msg.transform.rotation.x = frame_data['rotation']['x']
        msg.transform.rotation.y = frame_data['rotation']['y']
        msg.transform.rotation.z = frame_data['rotation']['z']
        msg.transform.rotation.w = frame_data['rotation']['w']
    else:
        msg.header.frame_id = frame_id
        msg.transform.rotation.w = 1.0
    msg.header.stamp = rospy.Time.now()
    staticBroadcaster.sendTransform(msg)


# Wrap camera requirements
class Camera():
    __slots__ = 'cvbridge', 'camera_info_msg', 'image_pub', 'info_pub'
    def __init__(self, topic_base, frame_id, intrinsic):
        self.cvbridge = cv_bridge.CvBridge()
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = frame_id
        self.camera_info_msg.K = intrinsic
        projection = np.zeros([3,4])
        projection[:3,:3] = np.array(intrinsic).reshape(3,3)
        self.camera_info_msg.P = projection.reshape(12)
        self.camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.image_pub = rospy.Publisher(os.path.join(topic_base, 'image_raw'), Image, queue_size=10)
        self.info_pub = rospy.Publisher(os.path.join(topic_base, 'camera_info'), CameraInfo, queue_size=10)

    def publish(self, stamp, image, encoding='passthrough'):
        # Update header time
        self.camera_info_msg.header.stamp = stamp

        # Publish image
        image_raw_msg = self.cvbridge.cv2_to_imgmsg(image, encoding)
        image_raw_msg.header = self.camera_info_msg.header
        self.image_pub.publish(image_raw_msg)

        # Publish info
        self.camera_info_msg.height = image_raw_msg.height
        self.camera_info_msg.width = image_raw_msg.width
        self.info_pub.publish(self.camera_info_msg)    


# Loads the data and plays it out onto the topics provided in the config.yaml file within the carla-data folder.
rospy.init_node('playCarlaData', anonymous=True)

# Get arguments
argv = rospy.myargv(argv=sys.argv)
try:
    datapath = argv[1]
except:
    print("Please provide arguments:")
    print("carla_data_folder")
    exit()

# Load the data
dataloader = CarlaDataLoader(datapath, lidar=False, groundtruth=False, logdepth=True)

# Load the configuration
with open(os.path.join(datapath, 'config.yaml')) as file:
    config = yaml.load(file, Loader=yaml.FullLoader)

    # Setup the important static frames
    staticBroadcaster = tf2_ros.StaticTransformBroadcaster()
    AddStaticFrame(staticBroadcaster, config['world_frame_id'])
    for frame_id, frame_data in config['joints'].items():
        AddStaticFrame(staticBroadcaster, frame_id, frame_data)

    # Setup the cameras
    cam = config['color_camera']
    color_camera = Camera(cam['topic'], cam['frame_id'], cam['intrinsic'])
    cam = config['depth_camera']
    depth_camera = Camera(cam['topic'], cam['frame_id'], cam['intrinsic'])

    # Extra
    rate = config['publish_rate']
    base_frame_id = config['base_frame_id']
    world_frame_id = config['world_frame_id']

# Setup the moving frame
# TF broadcaster
tf_pub = tf2_ros.TransformBroadcaster()
t = TransformStamped()
t.header.frame_id = world_frame_id
t.child_frame_id = base_frame_id

# Wait for user input to run
input("Press Enter to continue...")

# Setup the timer
timer = rospy.Rate(rate)

pose, rgb, depth, _ = dataloader.nextSetOfData()
i = 0
while pose is not None and rgb is not None and depth is not None:
    if rospy.is_shutdown():
        break

    # Show the frame we're publishing
    print("Publishing frame", i)
    i += 1

    # Publish the transform frame
    stamp = rospy.Time.now()
    quat = R.from_matrix(pose.rotation.T).as_quat()
    t.header.stamp = stamp
    t.transform.translation = Vector3(*pose.location)
    t.transform.rotation = Quaternion(*quat)
    tf_pub.sendTransform(t)
    # Sleep for a fraction of the rate so the tf exists
    rospy.sleep((1.0/rate)/4)

    # Publish the RGB image
    depth = depth.astype(np.float32)
    # depth = depth.astype(np.float32)*1000
    # small fix to make the data look better
    sigma = 0.002**0.5
    depth = depth + np.random.normal(0,sigma,depth.shape).reshape(depth.shape).astype(np.float32)
    # depth[depth>=9] = np.Inf
    # depth[depth<1] = -np.Inf
    color_camera.publish(stamp, rgb, encoding='rgb8')
    depth_camera.publish(stamp, depth, encoding='32FC1')

    # Get the next data
    pose, rgb, depth, _ = dataloader.nextSetOfData()
    timer.sleep()

if not rospy.is_shutdown():
    print("Spinning to keep frames alive...")
    rospy.spin()
