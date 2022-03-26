# Simple utility to spin up multiple static TF's from a the parameter server

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

rospy.init_node('StaticTFLinker', anonymous=True)

# Get the frames from the parameter server
frames = rospy.get_param("joints/", None)
if frames is None or len(frames) == 0:
    rospy.loginfo("No static frames to spin, exiting...")
    exit()

# TF broadcaster
pub = tf2_ros.StaticTransformBroadcaster()

try:
    # Iterate through all frames
    for frame_id, frame_data in frames.items():
        msg = TransformStamped()
        msg.child_frame_id = frame_id
        msg.header.frame_id = frame_data['parent']
        msg.header.stamp = rospy.Time.now()
        msg.transform.translation.x = frame_data['translation']['x']
        msg.transform.translation.y = frame_data['translation']['y']
        msg.transform.translation.z = frame_data['translation']['z']
        msg.transform.rotation.x = frame_data['rotation']['x']
        msg.transform.rotation.y = frame_data['rotation']['y']
        msg.transform.rotation.z = frame_data['rotation']['z']
        msg.transform.rotation.w = frame_data['rotation']['w']
        pub.sendTransform(msg)
except:
    rospy.logerr("Error in creating static frames. Exiting...")
    exit()

# Spin until killed
rospy.loginfo("Spinning all linked static TF frames until killed")
rospy.spin()
