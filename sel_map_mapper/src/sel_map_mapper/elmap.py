from copyreg import pickle
import os
from numpy import save
import yaml
from sel_map_segmentation import ColorScale
import rospy
from mesh_msgs.msg import MeshGeometryStamped, MeshGeometry, MeshMaterial, \
                            MeshMaterials, MeshMaterialsStamped, MeshFaceCluster, \
                            MeshVertexCostsStamped, MeshVertexCosts
from mesh_msgs.srv import GetMaterials, GetMaterialsResponse
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import uuid
# from sel_map_mesh import TriangularMesh
# from sel_map_segmentation import CameraSensor
from sel_map_mesh_publisher import TriangularMeshPublisher, roscpp_init, roscpp_shutdown
import numpy as np
import threading
# import open3d as o3d
import message_filters
import pickle
from pathlib import Path
from copy import deepcopy, copy

from tf.transformations import quaternion_from_matrix

# Helper classes to slightly speed up rospy
class Point():
    __slots__ = __fields__ = 'x', 'y', 'z'
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

class MeshTriangleIndices():
    __slots__ = __fields__ = 'vertex_indices'
    def __init__(self, vertex_indices=np.zeros(3)):
        self.vertex_indices = vertex_indices

# Pose object
class Pose():
    __slots__ = 'location', 'rotation'
    def __init__(self, location=np.zeros(3), rotation=np.eye(3)):
        self.location = location
        self.rotation = rotation

# Frame object
class Frame():
    __slots__ = 'id', 'timestamp', 'origin'
    def __init__(self, id=0, timestamp=0, origin=Pose()):
        self.id = id; self.timestamp = timestamp; self.origin = origin

# Frame object
class SaveOptions():
    __slots__ = 'dir', 'classifications', 'confidence'
    def __init__(self, dir=None, classifications=True, confidence=False):
        self.dir = dir; self.classifications = classifications; self.confidence = confidence

class Map:
    __slots__ = 'uuid', 'enableMat', 'saveOpts', 'thresholdElemToMove', 'timestamp', \
                'pub_mesh', 'pub_costs', 'pub_camera', 'frame', 'mesh', 'origin', 'cached_origin', 'camera', 'lock', \
                'vertices', 'normals', 'simplices', 'classes', 'meshMsg', 'heightCosts', 'pub_tf', 'map_tf_msg', \
                'mesh_mat_service', 'materials', 'daemon_thread', 'thread_rate', 'save', 'publish', 'threaded', 'crash'
    def __init__(self, mesh, camera=None, mesh_topic='mesh', mat_service='get_materials', enableMat = False, saveOpts=SaveOptions(), thresholdElemToMove=1, threadRate=20, threaded=True, mapCameraPosePublisher=None):
        # start roscpp
        roscpp_init()
        self.uuid = str(uuid.uuid4())
        self.enableMat = enableMat

        self.saveOpts = saveOpts
        if saveOpts.dir is not None:
            print("Saving Enabled")
            print("Save location:", os.path.realpath(saveOpts.dir))
            Path(os.path.join(saveOpts.dir, 'vertices/')).mkdir(parents=True, exist_ok=True)
            Path(os.path.join(saveOpts.dir, 'faces/')).mkdir(parents=True, exist_ok=True)
            if saveOpts.classifications:
                Path(os.path.join(saveOpts.dir, 'classes/')).mkdir(parents=True, exist_ok=True)
            if saveOpts.confidence:
                Path(os.path.join(saveOpts.dir, 'confidence/')).mkdir(parents=True, exist_ok=True)
            if not saveOpts.classifications and not saveOpts.confidence:
                raise Exception('Must save with either classes or confidence!')
            # Make sure we delete any prior save
            try:
                os.remove(os.path.join(saveOpts.dir, "save_dict.pkl"))
            except OSError:
                pass

        self.thresholdElemToMove = thresholdElemToMove
        # self.pub_mesh = rospy.Publisher(mesh_topic, MeshGeometryStamped, queue_size=10)
        # self.time_delay = rospy.Duration(0)
        self.pub_costs = rospy.Publisher(mesh_topic + '/costs', MeshVertexCostsStamped, queue_size=10)
        self.pub_camera = None
        if mapCameraPosePublisher is not None:
            self.pub_camera = rospy.Publisher(mapCameraPosePublisher, PoseStamped, queue_size=10)
        import tf2_ros
        self.pub_tf = tf2_ros.StaticTransformBroadcaster()
        # self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)
        world_base = rospy.get_param("world_base", "odom")
        self.map_tf_msg = TransformStamped()
        self.map_tf_msg.header.frame_id = world_base
        self.map_tf_msg.child_frame_id = "sel_map"
        self.map_tf_msg.transform.rotation.x = 0
        self.map_tf_msg.transform.rotation.y = 0
        self.map_tf_msg.transform.rotation.z = 0
        self.map_tf_msg.transform.rotation.w = 1
        self.map_tf_msg.header.stamp = rospy.Time.now()# - self.time_delay
        self.pub_tf.sendTransform(self.map_tf_msg)
        self.frame = Frame()

        # def syncCallback(msg_in, self):
        #     self.time_delay = rospy.Time.now() - msg_in.transforms[0].header.stamp

        # tf_timer = message_filters.Subscriber('/tf', TFMessage)
        # tf_timer.registerCallback(syncCallback, self)

        # Create the mesh map
        self.mesh = mesh
        # self.mesh.create()
        self.origin = Pose(np.zeros(3), np.eye(3))
        # The cached origin used for the pub_camera publisher
        self.cached_origin = Pose(np.zeros(3), np.eye(3))

        # Segmentation network
        self.camera = camera
        if camera is None:
            self.camera = CameraSensor()

        # For threading the publisher seperately
        self.lock = threading.Lock()

        # Prepped Mesh Message
        # self.vertices, self.normals, self.simplices = self.mesh.getMesh()
        # self.classes = self.mesh.getClasses()
        self.vertices, self.simplices, self.classes = self.mesh.getTrimmed()
        self.meshMsg = MeshGeometryStamped()
        self.meshMsg.uuid = self.uuid
        self.meshMsg.header = Header(frame_id='sel_map',stamp=rospy.Time.now())
        # self.meshMsg.mesh_geometry.faces = [MeshTriangleIndices(idxs) for idxs in self.simplices]
        self.pub_mesh = TriangularMeshPublisher(mesh, self.uuid, "sel_map", mesh_topic)
        
        self.heightCosts = MeshVertexCostsStamped()
        self.heightCosts.header = self.meshMsg.header
        self.heightCosts.uuid = self.uuid
        self.heightCosts.type = 'height'
        # self.heightCosts.mesh_vertex_costs = MeshVertexCosts(costs=self.vertices[:,2]-np.min(self.vertices[:,2]))

        # Handle Materials
        self.mesh_mat_service = None
        self.materials = None # Prepped callback
        if enableMat:
            # ROS service
            if not threaded:
                self.mesh_mat_service = rospy.Service(mat_service, GetMaterials, self.materialCallback)
            # Preprocess material colors
            self.materials = MeshMaterials()
            import rospkg
            rospack = rospkg.RosPack()
            path = rospack.get_path('sel_map')
            property_file = rospy.get_param("terrain_properties", os.path.join(path, "config/semseg_properties/csail_semseg_properties.yaml"))
            # property_file = "config/fastSCNN_properties_citys.yaml"
            with open(property_file) as file:
                prop_dict = yaml.load(file, Loader=yaml.FullLoader)
                keys_list = list(prop_dict)
            colorscale_args = rospy.get_param("colorscale", None)
            if colorscale_args is None:
                with open(os.path.join(path, "config/colorscales/default.yaml")) as file:
                    colorscale_args = yaml.load(file, Loader=yaml.FullLoader)['colorscale']
            colorscale = ColorScale(args=colorscale_args)
            if colorscale.bypass:
                # we're not using the colorscale here, so use properties instead
                materials = [ColorRGBA(float(prop_dict[key]['color']['R'])/255.0,
                                    float(prop_dict[key]['color']['G'])/255.0,
                                    float(prop_dict[key]['color']['B'])/255.0,1) for key in keys_list]
            else:
                # Map the friction to the colors
                materials = [ColorRGBA(*colorscale.mapToColor(prop_dict[key]['friction']['mean']),1) for key in keys_list]
            self.materials.materials = [MeshMaterial(color=materials[i], has_texture=False) for i in range(len(materials))]
            self.materials.cluster_materials = np.array(range(len(materials)))
        
        self.threaded = threaded
        self.daemon_thread = None
        self.thread_rate = threadRate
        self.publish = False
        self.save = False
        self.crash = False
        if threaded:
            # Start a daemon thread
            self.daemon_thread = threading.Thread(target=self.threadedRoutine, args=[mat_service])
            self.daemon_thread.daemon = True 
            self.daemon_thread.start()

    def __del__(self):
        # kill roscpp
        roscpp_shutdown()
        # if self.threaded and not self.crash:
        #     self.daemon_thread.do_run = False
        #     self.daemon_thread.join()
        del self.pub_mesh

    def threadedRoutine(self, mat_service):
        try:
            t = threading.currentThread()
            published = False
            # start the mesh material service if needed
            if self.enableMat:
                self.mesh_mat_service = rospy.Service(mat_service, GetMaterials, self.materialCallback)
            
            # run at rate
            timer = rospy.Rate(self.thread_rate)
            while getattr(t, "do_run", True) and not rospy.is_shutdown():
                if self.publish:
                    published = True
                if self.publish or self.save:
                    self.publishSaveMap()
                if published:
                    self.heightCosts.header.stamp = rospy.Time.now()
                    self.pub_costs.publish(self.heightCosts)
                    # self.map_tf_msg.header.stamp = rospy.Time.now()# - self.time_delay
                    # self.pub_tf.sendTransform(self.map_tf_msg)
                timer.sleep()
            roscpp_shutdown()
        except Exception as e:
            rospy.logerr(e)
            self.crash = True
    
    def materialCallback(self, req):
        if (req.uuid != self.uuid):
            return
        msg = MeshMaterialsStamped()
        msg.uuid = self.uuid
        msg.header = Header(frame_id='sel_map',stamp=rospy.Time.now())
        with self.lock:
            msg.mesh_materials = self.materials
            return GetMaterialsResponse(msg)
 
    def shiftIfNeeded(self, pose):
        '''
        Shifts the mesh if the camera has moved past a certain threshold # of elements
        '''
        threshold = self.mesh.elementLength * self.thresholdElemToMove
        displacement = pose.location - self.origin.location	
        # short circuit if the shift isn't enough
        if np.abs(displacement[0]) < threshold and np.abs(displacement[1]) < threshold:
            return
        
        # shift the mesh
        elementShift = np.ceil((np.abs(displacement[:2]) - threshold) / self.mesh.elementLength)
        elementShift[elementShift<0] = 0
        elementShift = elementShift * -np.sign(displacement[:2])
        self.mesh.shiftMeshElements(elementShift[0], elementShift[1])
        # calculate a new origin
        self.origin.location = self.origin.location.copy()
        self.origin.location[:2] -= elementShift * self.mesh.elementLength
        
    def update(self, pose, rgbd, intrinsic=None, R=None, min_depth=0, max_depth=5.0):
        if self.crash:
            raise Exception('Map Daemon Thread Crashed!')
        # Shift the mesh if we need to
        self.shiftIfNeeded(pose)

        # Get the segmented points (rgbd is a tuple of (rgb, depth))
        poseCameraToMap = Pose()
        poseCameraToMap.location = pose.location - self.origin.location
        poseCameraToMap.rotation = pose.rotation
        self.camera.setPoseCameraToMap(poseCameraToMap)
        self.camera.updateSensorMeasurements(rgbd[0], rgbd[1])
        points = self.camera.getProjectedPointCloudWithLabels(intrinsic=intrinsic, R=R, min_depth=min_depth, max_depth=max_depth)
        # Shift and rotate as needed
        points[:,:3] = pose.location + np.dot(points[:,:3], np.transpose(pose.rotation))
        
        # Transform to map origin
        points[:,:3] = points[:,:3] - self.origin.location

        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(points[:,:3])
        # axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.001, origin=[0, 0, 0])
        # o3d.visualization.draw_geometries([pcd, axes])
        
        # Advance and clean the map (lazy can be true if the points pushed to the map is relatively constant)
        self.mesh.advance(classpoints=points, z_height=pose.location[2])
        self.mesh.clean(lazy=True)

        # Publish the camera pose synchronously
        if self.pub_camera is not None:
            poseMsg = PoseStamped()
            poseMsg.header = Header(frame_id='sel_map',stamp=rospy.Time.now())
            poseMsg.pose.position.x = pose.location[0]# - self.frame.origin.location[0]
            poseMsg.pose.position.y = pose.location[1]# - self.frame.origin.location[1]
            poseMsg.pose.position.z = pose.location[2]# - self.frame.origin.location[2]
            rot = np.identity(4)
            rot[:3,:3] = pose.rotation
            quaterion = quaternion_from_matrix(rot)
            poseMsg.pose.orientation.x = quaterion[0]
            poseMsg.pose.orientation.y = quaterion[1]
            poseMsg.pose.orientation.z = quaterion[2]
            poseMsg.pose.orientation.w = quaterion[3]
            self.pub_camera.publish(poseMsg)

    def queueSavePublish(self, save=False, publish=False, timestamp=0):
        if self.crash:
            raise Exception('Map Daemon Thread Crashed!')
        # Helper to copy data
        def copyData():
            # vertices, normals, _ = self.mesh.getMesh()
            # classes = self.mesh.getClasses()
            # self.vertices = vertices.copy()
            # self.normals = normals.copy()
            # self.classes = [el.copy() for el in classes]
            vertices, simplices, classes = self.mesh.getTrimmed()
            self.vertices = vertices.copy()
            self.simplices = simplices.copy()
            self.classes = [cl for cl in classes]
            self.frame.timestamp = timestamp
            self.frame.origin = copy(self.origin)
            # Tell other thread that we are a go
            self.save = save or self.save
            self.publish = publish or self.publish
            # set the time
            # self.meshMsg.header.stamp = rospy.Time.now()

        # If we can lock our internal stores, then do so
        if (self.lock.acquire(False)):
            copyData()
            self.lock.release()
        # If we can't, then see if the state changes. if anything becomes true, then wait
        elif (not self.save and save) or (not self.publish and publish):
            with self.lock:
                copyData()
        
        # if we're not threaded, then run the publishSave routine now
        if not self.threaded:
            self.publishSaveMap()

    def publishSaveMap(self):
        # frame = deepcopy(self.frame)
        with self.lock:
            if self.save:
                rospy.loginfo("[sel_map] Saving...")
                import time
                tic = time.perf_counter()
                # save the geometry
                vertex_file = os.path.join(self.saveOpts.dir,'vertices/' + str(self.frame.id) + '.npy')
                simplex_file = os.path.join(self.saveOpts.dir,'faces/' + str(self.frame.id) + '.npy')
                np.save(vertex_file, self.vertices)
                np.save(simplex_file, self.simplices)

                # Save classes and confidence as desired
                if self.saveOpts.classifications:
                    classes_file = os.path.join(self.saveOpts.dir,'classes/' + str(self.frame.id) + '.npy')
                    np.save(classes_file, self.pub_mesh.get_onehot_classes(self.classes, passthrough=False))
                if self.saveOpts.confidence:
                    confidence_file = os.path.join(self.saveOpts.dir,'confidence/' + str(self.frame.id) + '.npz')
                    np.savez(confidence_file, self.classes)

                # Save the frame reference pickle
                with open(os.path.join(self.saveOpts.dir, "save_dict.pkl"), 'ab+') as fp:
                    pickle.dump(self.frame, fp)
                self.frame.id += 1
                toc = time.perf_counter()
                print("save time", toc-tic)

            if self.publish:
                rospy.loginfo("[sel_map] Publishing...")
                if self.enableMat:
                    # First prepare the new colors
                    # simplex_colors = np.argmax(self.classes, axis=1)
                    simplex_colors = self.pub_mesh.get_onehot_classes(self.classes, passthrough=False)

                    # not actually clustered, just for viz
                    clustered_faces = [MeshFaceCluster(face_indices=np.argwhere(simplex_colors==i).flatten().astype(np.uint32)) for i in self.materials.cluster_materials]

                    # update the message for the service
                    self.materials.clusters = clustered_faces
                
                # Process the message
                # vertices = self.vertices[:,:3]
                # self.meshMsg.mesh_geometry.vertices = [Point(*vert) for vert in vertices]
                # self.meshMsg.mesh_geometry.vertex_normals = [Point(*norm) for norm in self.normals]
                # self.meshMsg.header.stamp = rospy.Time.now()
                # # Publish
                # self.pub_mesh.publish(self.meshMsg)
                # Get costs
                cost = self.vertices[:,2]-self.mesh.originHeight
                cost[cost<-1] = -1
                self.heightCosts.mesh_vertex_costs = MeshVertexCosts(costs=cost)
                import time
                tic = time.perf_counter()
                self.vertices[:,:2] += self.frame.origin.location[:2]
                self.pub_mesh.publish(self.vertices, self.simplices)
                toc = time.perf_counter()
                print("publish time", toc-tic)
                # Cache the origin
                # self.cached_origin.location = frame.origin.location
                # Map transform
                # self.map_tf_msg.transform.translation.x = self.frame.origin.location[0]
                # self.map_tf_msg.transform.translation.y = self.frame.origin.location[1]
                # self.map_tf_msg.transform.translation.z = self.frame.origin.location[2]
                # self.pub_tf.publish(TFMessage([self.map_tf_msg]))
                # Get costs
                # self.heightCosts.mesh_vertex_costs = MeshVertexCosts(costs=self.vertices[:,2])
                # self.pub_tf.sendTransform(self.map_tf_msg)
            self.save = self.publish = False
    
