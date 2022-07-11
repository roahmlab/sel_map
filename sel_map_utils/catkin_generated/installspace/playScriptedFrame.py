# Simple utility to spin up multiple static TF's from a the parameter server

import rospy
import sys
import yaml
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation as R
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

rospy.init_node('PlayScriptedFrame', anonymous=True)

# config
rate = 20

# Get arguments
argv = rospy.myargv(argv=sys.argv)
try:
    script_file = argv[1]
    sync_topic = argv[2]
except:
    print("Please provide arguments:")
    print("script_file sync_topic")
    exit()

# Process the script file
with open(script_file) as file:
    cfg = yaml.load(file, Loader=yaml.FullLoader)

    # Get the pose script
    script_len = (len(cfg['poses'])-1)*cfg['pose_interval']
    list_times = np.linspace(0, script_len, len(cfg['poses']), endpoint=True)
    times = np.linspace(0, script_len, script_len*rate+1, endpoint=True)
    pose_list = np.array([np.fromstring(values, sep=',') for values in cfg['poses']])
    
    # Isolate the translations and euler angles
    translations = pose_list[:,0:3]
    euler_angles = pose_list[:,3:6]

    # Generate the interpolation and smoothing
    sigma = cfg['smoothing_sigma']
    trans_filt = np.zeros((len(times),3))
    trans_filt[:,0] = gaussian_filter1d(np.interp(times, list_times, translations[:,0]), sigma)
    trans_filt[:,1] = gaussian_filter1d(np.interp(times, list_times, translations[:,1]), sigma)
    trans_filt[:,2] = gaussian_filter1d(np.interp(times, list_times, translations[:,2]), sigma)
    translations = trans_filt

    ang_filt = np.zeros((len(times),3))
    ang_filt[:,0] = gaussian_filter1d(np.interp(times, list_times, euler_angles[:,0]), sigma)
    ang_filt[:,1] = gaussian_filter1d(np.interp(times, list_times, euler_angles[:,1]), sigma)
    ang_filt[:,2] = gaussian_filter1d(np.interp(times, list_times, euler_angles[:,2]), sigma)
    orientations = R.from_euler('zyx', ang_filt, degrees=True).as_quat()

print("Awaiting sync topic", sync_topic)
# Wait for the sync topic
rospy.wait_for_message(sync_topic, rospy.AnyMsg)
timer = rospy.Rate(rate)

# TF broadcaster
pub = tf2_ros.TransformBroadcaster()
t = TransformStamped()
t.header.frame_id = cfg['parent']
t.child_frame_id = cfg['frame_name']

# Iterate through all frames at rate
for i in range(len(times)):
    print("Publishing", i)
    if rospy.is_shutdown():
        break
    
    # Build the transform and publish
    t.header.stamp = rospy.Time.now()
    t.transform.translation = Vector3(*translations[i,:])
    t.transform.rotation = Quaternion(*orientations[i,:])
    pub.sendTransform(t)
    
    # Keep rate
    timer.sleep()
