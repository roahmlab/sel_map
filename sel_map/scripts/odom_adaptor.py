import tf2_ros
import rospy
import numpy as np
import message_filters

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion, PoseStamped, Pose, Point
from nav_msgs.msg import Odometry

rospy.init_node('RealSenseTF')

tfBuffer = tf2_ros.Buffer()
tf_sub = tf2_ros.TransformListener(tfBuffer)

cam_pose_pub = rospy.Publisher('/cam_pose', PoseStamped, queue_size = 1)


def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q


def transform_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/TransformStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return transform_to_pq(msg.transform)

def callback(msg_in):
    frame_from = 'camera_link'
    frame_to = 'camera_color_optical_frame'
    try:
        # temp = tfBuffer.lookup_transform('odom', 'body', rospy.Time(0))
        # (trans,rot) = transform_stamped_to_pq(temp)
        # trans = [0,0,0]
        # rot = [0,0,0,1]
        trans = np.array([0.120, 0.0, -0.0705])
        rot = np.array([0.0, 0.258819, 0.0, 0.9659258])
        # temp = tfBuffer.lookup_transform('odom', 'back', rospy.Time(0))
        # (trans_depth,rot_depth) = transform_stamped_to_pq(temp)
        temp = tfBuffer.lookup_transform(frame_to, frame_from, rospy.Time(0))
        (trans_robot,rot_robot) = transform_stamped_to_pq(temp)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("error")
        return

    header = msg_in.header
    pose = msg_in.pose.pose

    pose.position.x += trans_robot[0] + trans[0]
    pose.position.y += trans_robot[1] + trans[1]
    pose.position.z += trans_robot[2] + trans[2]

    orient = np.array([pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w])

    # rot_correction = R.from_euler('ZY', [90], degrees=True).as_matrix()
    rot_cam = np.dot(R.from_quat(orient).as_matrix(),R.from_quat(rot).as_matrix())
    # rot_cam = np.dot(R.from_quat(rot).as_matrix(),rot_cam)
    # rot_cam = np.dot(rot_cam,rot_correction)
    rot_cam = R.from_matrix(rot_cam).as_quat()

    pose.orientation.x = rot_cam[0]
    pose.orientation.y = rot_cam[1]
    pose.orientation.z = rot_cam[2]
    pose.orientation.w = rot_cam[3]
    
    cam_pose_msg = PoseStamped()
    cam_pose_msg.header = header
    cam_pose_msg.header.frame_id = "odom"
    cam_pose_msg.pose = pose
    print(cam_pose_msg)
    cam_pose_pub.publish(cam_pose_msg)


odom_sub = message_filters.Subscriber('/odometry/imu', Odometry, queue_size=1)
odom_sub.registerCallback(callback)


# # Four publishers for four cameras
# # tf_pub = rospy.Publisher('tf_intermediate', JointState, queue_size = 10)
# tf_pub = rospy.Publisher('tf_intermediate', TransformStamped, queue_size = 10)
# # depth_pose_pub = rospy.Publisher('rear_depth_pose', PoseStamped, queue_size = 10)
# cam_pose_pub = rospy.Publisher('realsCamPose', PoseStamped, queue_size = 10)


# rate = rospy.Rate(10.0)
# while not rospy.is_shutdown():
#     try:
#         # temp = tfBuffer.lookup_transform('odom', 'body', rospy.Time(0))
#         # (trans,rot) = transform_stamped_to_pq(temp)
#         # trans = [0,0,0]
#         # rot = [0,0,0,1]
#         # temp = tfBuffer.lookup_transform('odom', 'back', rospy.Time(0))
#         # (trans_depth,rot_depth) = transform_stamped_to_pq(temp)
#         temp = tfBuffer.lookup_transform('odom', 'lidar_body', rospy.Time(0))
#         (trans_robot,rot_robot) = transform_stamped_to_pq(temp)
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         # print("error")
#         continue
#     print(temp)

#     # location and rotation of base
#     # rotation1 = np.array([rot[0], rot[1], rot[2], rot[3]])
#     # rotation2 = np.array([-0.177, 0.193, 0.683, 0.682])
#     # rot1 = R.from_quat(rotation1)
#     # rot2 = R.from_quat(rotation2)
#     # rotat = np.dot(rot1.as_matrix(), rot2.as_matrix())
#     # rotation = R.from_matrix(rotat)
#     # test_rot = rotation.as_quat()
#     # rotation = test_rot

#     # location = trans + np.dot(rotat, np.transpose(np.array([-0.033, 0.099, 0.017])))

#     # print(trans_robot)
#     # print(rot_robot)
#     # trans correction 
#     trans_cam = trans_robot+ np.array([-0.007 ,0.095,-0.08])
#     # rotation correction
#     rot_correction = R.from_euler('ZY', [90,45], degrees=True).as_matrix()
#     rot_cam = np.dot(R.from_quat(rot_robot).as_matrix(),rot_correction)
#     rot_cam = R.from_matrix(rot_cam).as_quat()
#     # rot_cam = np.dot(R.from_quat(rot_cam).as_matrix(), rot_correction)
#     # rot_cam = R.from_matrix(rot_cam).as_quat()

#     # tf_msg = TransformStamped()
#     # tf_msg.header = header
#     # tf_msg.header.frame_id = "odom"
#     # tf_msg.child_frame_id = "map"
#     # tf_msg.transform.translation = Vector3(x=trans[0],y=trans[1],z=trans[2])
#     # tf_msg.transform.rotation = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])

#     # depth_pose_msg = PoseStamped()
#     # depth_pose_msg.header = header
#     # depth_pose_msg.header.frame_id = "odom"
#     # depth_pose_msg.pose.position = Point(*trans_depth)
#     # # pose_msg.pose.orientation = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])
#     # depth_pose_msg.pose.orientation = Quaternion(*rot_depth)

#     cam_pose_msg = PoseStamped()
#     cam_pose_msg.header = header
#     cam_pose_msg.header.frame_id = "odom"
#     cam_pose_msg.pose.position = Point(*trans_cam)
#     # pose_msg.pose.orientation = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])
#     cam_pose_msg.pose.orientation = Quaternion(*rot_cam)

#     # tfSensor.transform.translation = Vector3(x=-0.033, y=0.099, z=0.017)
#     # tfSensor.transform.rotation = Quaternion(x=-0.177, y=0.193, z=0.683, w=0.682)


#     # print("Publishing")
 
#     # tf_pub.publish(tf_msg)
#     # depth_pose_pub.publish(depth_pose_msg)
#     cam_pose_pub.publish(cam_pose_msg)
#     # rate.sleep()

rospy.spin()

