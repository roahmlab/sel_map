

from matplotlib import projections
# from pandas import array
import rospy
import message_filters
from mesh_msgs.msg import MeshGeometryStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point32, Point, PoseStamped
from moveit_msgs.msg import OrientedBoundingBox as Obb_msg
from tf2_msgs.msg import TFMessage
import tf2_ros
from std_msgs.msg import ColorRGBA
# from perception_open3d
# import open3d_conversions
from mpl_toolkits.mplot3d import Axes3D



import numpy  as np
import cv2
# import numpy_quaternion as n_quat
import matplotlib.pyplot as plt

from sklearn.cluster import AgglomerativeClustering
from sklearn.metrics import pairwise_distances
from scipy.spatial import distance_matrix, ConvexHull
from scipy.spatial.transform import Rotation

import open3d



def callback(mesh,tfBuffer ,pub, viz_pub,viz_pose):
    meshPoints = np.array([np.asarray([vertex.x,vertex.y,vertex.z]) for vertex in mesh.mesh_geometry.vertices])
    # print(meshPoints.shape)
    # for spot obsatcel height threshold set as - 5.3
    
    try:
        trans = tfBuffer.lookup_transform('odom', 'gpe', rospy.Time())
        # sel_map = tfBuffer.lookup_transform('odom', 'sel_map', rospy.Time())
        # print(sel_map.transform.rotation)
        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): 
        return
    meshPoints_thres = meshPoints[meshPoints[:,2] > -5.29]

    print("id: ", mesh.header.seq, "; point: ", np.amax(meshPoints[:,2]))
    # print(meshPoints2D.shape)
    print("transform gpe,", trans.transform.translation.z)
    # ground wrt to robot frame
    ground_ = trans.transform.translation.z 

    height_thres = ground_ + 0.025     # 25cm
    # print(height_thres)

    # meshPoints_thres = meshPoints[meshPoints[:,2] > height_thres]

    try:

        model = AgglomerativeClustering(n_clusters = None , linkage="single", affinity="euclidean", distance_threshold = 0.071)
        model.fit(meshPoints_thres[:,:2])
    except:
        print("Clustring error")
        return

    # print(model.labels_)
    
    hulls_2d = []
    hulls_2d_points = []
    clusters_points = []
    cluster_orignal = []
    obstacle_bbs = []
    for i in range(np.amax(model.labels_)+1):
        cluster = meshPoints_thres[model.labels_ ==i]
        # try:
        if len(cluster)< 5 :
            continue
        # # for oriented boinding box
        # hull_2d = ConvexHull(cluster[:,:2], qhull_options='QJ')
        # temp = cluster[hull_2d.vertices]
        # hulls_2d_points.append(temp[:,:2])
    
        # temp[:,2] = ground_
        
        # cluster= np.vstack((cluster, temp))
        # print(cluster)
        # print(np.amax(cluster[:,2]))

        
        # hulls_2d.append(temp[:,:2])
        
        # clusters_points.append(cluster)


        # for vertical bounding boxes
        cluster_orignal.append(cluster)
        temp1 = np.copy(cluster)
        temp2 = np.copy(cluster)
        temp1[:,2]= ground_
        # print(cluster)
        # print(np.amax(cluster[:,2]))
        temp2[:,2]= np.amax(cluster[:,2])
        cluster2D = np.vstack((temp1,temp2))
        clusters_points.append(cluster)
        # print(cluster)


    # print(len(obstacles))
        pcd = open3d.geometry.PointCloud(points=open3d.utility.Vector3dVector(cluster2D))
        obstacle_bbs.append(pcd.get_oriented_bounding_box( robust =True))
        print(obstacle_bbs)
        # except:
        #     continue


    # points = np.random.rand(50,3)
    # # print(points   )
    # # print(open3d.geometry.OrientedBoundingBox.create_from_points(open3d.utility.Vector3dVector(points)))
    

    
    
    markerarray = MarkerArray()
    # delmarker = Marker()

    # delmarker.action = delmarker.DELETEALL
    # markerarray.markers.append(delmarker)
    # viz_pub.publish(markerarray)
    # markerarray.markers=  []

    
    for i in range(len(obstacle_bbs)):

        box_points =np.asarray(obstacle_bbs[i].get_box_points())
        print(box_points)
















        Obb = Obb_msg()
        Obb_pose = PoseStamped()
        # ObbPoseViz = PoseStamped()
        Obb_pose.pose.position.x = obstacle_bbs[i].center[0]
        Obb_pose.pose.position.y = obstacle_bbs[i].center[1]
        Obb_pose.pose.position.z = obstacle_bbs[i].center[2]
        
        
        print("Obs - %u"%(i))
        
        rot = np.copy(obstacle_bbs[i].R)
        print(rot)
        print(np.linalg.det(rot))
        
        # print(orient)
        # if np.linalg.det(rot) < 0:         
        #     rot =  np.multiply(np.linalg.inv(np.copy(rot)),np.asarray([[-1], [-1.], [-1.] ]))
        orient = Rotation.from_matrix(rot).as_quat()
        # orient[:3] = -1*orient[:3]
        print(orient)
        print(np.linalg.norm(orient))
        # print(np.linalg.det(rot))
        print("......")

        
        Obb_pose.pose.orientation.x = orient[0]
        Obb_pose.pose.orientation.y = orient[1]
        Obb_pose.pose.orientation.z = orient[2]
        Obb_pose.pose.orientation.w = orient[3]

        Obb.pose = Obb_pose.pose
        # print(Obb.pose.position.x)
        # print(obstacle_bbs[i].extent)

        Obb.extents.x = obstacle_bbs[i].extent[0]
        Obb.extents.y = obstacle_bbs[i].extent[1]
        Obb.extents.z = obstacle_bbs[i].extent[2]
        # print(Obb.extents)
        pub.publish(Obb)
        # print(Obb)
        # for visaulization 
        
        # print(points)
        # print(clusters_points[i])
        # k = input()
        # if k == 65:
        # ax = plt.axes(projection= '3d')
        # ax.scatter3D(clusters_points[i][:,0],clusters_points[i][:,1],clusters_points[i][:,2], color='r')
        # ax.scatter3D(cluster_orignal[i][:,0],cluster_orignal[i][:,1],cluster_orignal[i][:,2], color='g')
        # ax.scatter3D(points[:,0],points[:,1],points[:,2], color='b')
        # plt.show()

        # 2d bounding 
        # ((x,y), (ext_x,ext_y), rot) = cv2.minAreaRect(clusters_points[i][:,:2])
        # print(((x,y), (ext_x,ext_y), rot))
        # print(np.array(np.reshape(hulls_2d_points[i], (hulls_2d_points[i].shape[0], 1, hulls_2d_points[i].shape[1]))))
        # k =np.array(np.reshape(hulls_2d_points[i], (hulls_2d_points[i].shape[0], 1, hulls_2d_points[i].shape[1])))

        # # plt.plot(hulls_2d_points[i][:,0],hulls_2d_points[i][:,1],'r')
        # # plt.show()
        # # plt.plot(clusters_points[i][:,0],clusters_points[i][:,1],'b')
        # # plt.show()
        # box =cv2.minAreaRect(k)
        # print(box)
        # print(points)


        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "sel_map"
        marker.header.seq = i
        Obb_pose.header = marker.header
    
        marker.lifetime.nsecs = 100
        marker.id = i
        marker.ns = "Obs - %u"%(i)
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = Obb.pose.position
        marker.pose.orientation = Obb.pose.orientation
        print(marker.header.stamp)
        print(marker.pose.orientation)
        # marker.pose.orientation.x = 0.
        # marker.pose.orientation.y = 0.
        # marker.pose.orientation.z = 0.
        # marker.pose.orientation.w = 1.
        marker.scale.x = Obb.extents.x +0.
        marker.scale.y = Obb.extents.y +0.
        marker.scale.z = Obb.extents.z +0.
        # marker.scale.x = 0.03
        # marker.scale.y = 0.03
        # marker.scale.z = 0.
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.frame_locked = True
        # for i in  range(box_points.shape[0]) :
        #     pt = Point()
        #     c= ColorRGBA()
        #     point = np.copy(box_points[i, :])
        #     # print(point)
        #     pt.x =  point[0]
        #     pt.y =  point[1]
        #     pt.z =  point[2]
        #     marker.points.append(pt)
        #     c.a = 0.8
        #     c.r = 1.0
        #     c.g = 0.0
        #     c.b = 0.0
        #     marker.colors.append(c)
        # print(marker.points)
        viz_pub.publish(marker)
        viz_pose.publish(Obb_pose)

    #     markerarray.markers.append(marker)
        
  
   
    # viz_pub.publish(markerarray)
        
        



def meshSub():
    rospy.init_node('boundingBoxPub', anonymous=True)
    mesh_sub = message_filters.Subscriber("/mesh", MeshGeometryStamped)
    tf_sub = message_filters.Subscriber("/tf", TFMessage)
    # rospy.Subscriber("/mesh", MeshGeometryStamped, callback)
    pub = rospy.Publisher('boundingBox3D', Obb_msg, queue_size=10)
    viz_pub = rospy.Publisher('vizBoundingBox', Marker, queue_size=10)
    viz_pose = rospy.Publisher('vizBoundingBoxPose', PoseStamped, queue_size=10)


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
      

    
    # trans = tfBuffer.__get_frames()



    ts = message_filters.TimeSynchronizer([mesh_sub], 10)
    ts.registerCallback(callback, tfBuffer, pub, viz_pub, viz_pose)


    rospy.spin()
if __name__ == '__main__':
    meshSub()