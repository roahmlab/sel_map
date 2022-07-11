import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from tf.transformations import quaternion_matrix
from sel_map_mapper import Pose, Mapper, SaveOptions
from sel_map_mesh import TriangularMesh
from sel_map_segmentation import CameraSensor
import numpy as np
import sys

from cv_bridge import CvBridge
import cv2
import tf2_ros

map = None
cv_bridge = None
firstPose = True
rotate = False
savingFlag = False
initialTime = None
save_interval = rospy.Duration(0,0)
last_save = rospy.Duration(0,0)
world_base = "odom"

# EXTRA
threaded = True
publish = True
tfBuffer = tf2_ros.Buffer()

def syncedCallback(rgb, depth, info, pose=None, meta=None):
    global map, cv_bridge, firstPose, rotate, savingFlag, initialTime, last_save, world_base
    # get poses ready #TODO bring robot back into picture
    if pose is None:
        try:
            tf_stamped = tfBuffer.lookup_transform(world_base, depth.header.frame_id, depth.header.stamp, rospy.Duration(0.01))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        location = tf_stamped.transform.translation
        location = np.array([location.x, location.y, location.z])
        # re-orient transform to match camera frame.
        rot = tf_stamped.transform.rotation
        rot = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        rot = rot[:3,:3] @ np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    else:
        # try:
        #     pose = pose.pose # Adapt for with covariance, but ignore that for now.
        # except:
        #     pass
        print(pose)
        rot = quaternion_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        rot = rot[:3,:3]
        location = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
    rospy.loginfo("[sel_map] Message received!")
    pose = Pose(location=location, rotation=rot)
    # Set initial pose.
    if firstPose:
        map.frame.origin.location[0:2] = location[0:2]
        initialTime = rospy.Time.now() 
        firstPose = False

    # get images ready using CVBridge
    rgb_np = cv_bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
    depth_np = cv_bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')

    # Account for openni format
    if depth_np.dtype == np.uint16:
        depth_np = depth_np.astype(np.float32) / 1000

    # rotate if needed
    if rotate:
        rgb_np = cv2.rotate(rgb_np, cv2.ROTATE_180)
        depth_np = cv2.rotate(depth_np, cv2.ROTATE_180)

    # Tuple for RGB-D
    rgbd = (rgb_np, depth_np)
    R = np.array([[0,0,1],[1,0,0],[0,1,0]]) #TODO

    intrinsic = None
    if info is not None:
        intrinsic = np.reshape(info.K, (3, 3))

    # Update the map
    # indoor data seems to have negative depth values resulting in a seg fault
    map.update(pose, rgbd, intrinsic=intrinsic, R=R, min_depth=0.5, max_depth=8.0)
    
    # Queue a new publish (without saving for now)
    if savingFlag:
        timestamp = (rospy.Time.now() - initialTime)
        interval = timestamp - last_save
        if interval >= save_interval:
            map.queueSavePublish(save=savingFlag, publish=publish, timestamp=timestamp)
            last_save = timestamp
        else:
            map.queueSavePublish(save=False, publish=publish)
    else:
        map.queueSavePublish(save=False, publish=publish)

def sel_map_node(mesh_bounds, elementLength, thresholdElemToMove):
    global map, cv_bridge, firstPose, rotate, savingFlag, save_interval, world_base

    rospy.init_node('sel_map')
    cv_bridge = CvBridge()
    tf2_ros.TransformListener(tfBuffer)

    # Get parameters
    num_cameras = rospy.get_param("num_cameras", 1) # TODO
    update_policy = rospy.get_param("update_policy", "fifo") # TODO
    sync_slop = rospy.get_param("sync_slop", 0.3)
    queue_size = rospy.get_param("queue_size", 2)
    point_limit = rospy.get_param("point_limit", 20)
    publish_rate = rospy.get_param("publish_rate", 5)
    enableMat = rospy.get_param("enable_mat_display", True)
    rotate = rospy.get_param("camera_flipped", False)
    save_mesh_location = rospy.get_param("save_mesh_location", "")
    save_classes = rospy.get_param("save_classes", True)
    save_confidence = rospy.get_param("save_confidence", False)
    save_interval = rospy.get_param("save_interval", 0)
    save_interval = rospy.Duration.from_sec(save_interval)
    world_base = rospy.get_param("world_base", "odom")

    cameras = rospy.get_param("cameras_registered", None)
    cameras = list(cameras.items())[0][1]
    print(cameras)

    saveOpts = SaveOptions()
    if len(save_mesh_location) > 0:
        saveOpts.dir = save_mesh_location
        saveOpts.classifications = save_classes
        saveOpts.confidence = save_confidence
        savingFlag = True

    # Prepare elements for the map
    segmentation_network = CameraSensor()
    mesh = TriangularMesh(bounds=np.asarray(mesh_bounds), elementLength=elementLength, pointLimit=point_limit, terrainClasses=segmentation_network.labels, heightPartition=10, heightSafetyCheck=0.2)
    mesh.create()
    map = Mapper(mesh, segmentation_network, thresholdElemToMove=thresholdElemToMove, threadRate=publish_rate, threaded=threaded, enableMat=enableMat, saveOpts=saveOpts)

    # Iterate over cameras TODO
    rgb_sub = message_filters.Subscriber(cameras["image_rectified"], Image)
    depth_sub = message_filters.Subscriber(cameras["depth_registered"], Image)
    info_sub = message_filters.Subscriber(cameras["camera_info"], CameraInfo)
    sub_list = [rgb_sub, depth_sub, info_sub]
    if "pose_with_covariance" in cameras \
        and cameras["pose_with_covariance"] is not None \
        and len(cameras["pose_with_covariance"]) > 0:
        pose_sub = message_filters.Subscriber(cameras["pose_with_covariance"], PoseWithCovarianceStamped)
        sub_list.append(pose_sub)
    elif "pose" in cameras \
        and cameras["pose"] is not None \
        and len(cameras["pose"]) > 0:
        pose_sub = message_filters.Subscriber(cameras["pose"], PoseStamped)
        sub_list.append(pose_sub)

    # Subscribe
    sync_sub = message_filters.ApproximateTimeSynchronizer(sub_list, queue_size=queue_size, slop=sync_slop)
    sync_sub.registerCallback(syncedCallback)
    rospy.loginfo("[sel_map] Callbacks registered, awaiting...")

    # Spin
    rospy.spin()
    rospy.loginfo("[sel_map] Shutting down.")

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) <= 5:
            print("Please provide mesh bounds, element size, and threshold to move")
            print("bound_x bound_y bound_z elem_size threshold_elem_move")
        mesh_bounds = [float(argv[1]), float(argv[2]), float(argv[3])]
        elem_size = float(argv[4])
        threshold = int(argv[5])
        sel_map_node(mesh_bounds, elem_size, threshold)
    except rospy.ROSInterruptException:
        del map
        pass
