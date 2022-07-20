import threading
import numpy as np
import rospy
import os
import yaml
import rospy
from mesh_msgs.msg import MeshGeometryStamped, MeshGeometry, MeshMaterial, \
                            MeshMaterials, MeshMaterialsStamped, MeshFaceCluster, \
                            MeshVertexCostsStamped, MeshVertexCosts
from mesh_msgs.srv import GetMaterials, GetMaterialsResponse
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, TransformStamped
from sel_map_segmentation import ColorScale
import uuid
import os
import yaml
import sys
import pickle
import tf2_ros

# IGNORE THE REST FOR NOW
argv = rospy.myargv(argv=sys.argv)
if len(argv) <= 3:
    print("Please provide save location, playback speed, playback decimation, and whether to perform synced playback or not")
    print("save_path playback_rate playback_decimation synced_playback_topic[optional] seek_ahead(s)[optional]")
path = argv[1]
playback_speed = float(argv[2])
playback_decimation = int(argv[3])
synced_playback = None
seek_ahead = 0
try:
    synced_playback = argv[4]
    seek_ahead = float(argv[5])
except:
    pass

if synced_playback is not None:
    rate = 20
else:
    rate = playback_speed

# datadir = 'output/ground-truth'
# datadir = '/home/pewen/temp/ours'

class PlaybackLoader:
    def __init__(self, path):
        self.path = path
        self.save_dict = open(os.path.join(path, "save_dict.pkl"), 'rb')
        self.classes = os.path.exists(os.path.join(path, "classes"))
        self.confidence = os.path.exists(os.path.join(path, "confidence"))
        self.done = False
        # Load the initial frame
        self.frame = None
        # try:
        #     self.frame = pickle.load(self.save_dict)
        # except:
        #     self.done = True

    def nextFrame(self):
        # Pull the next frame
        try:
            if self.frame is None:
                print(self.save_dict)
                self.frame = pickle.load(self.save_dict)
                return True
            if self.done == True:
                return False
            for _ in range(playback_decimation):
                self.frame = pickle.load(self.save_dict)
        except Exception as e:
            print(e)
            self.done = True
            return False
        return True

    def nextSetOfData(self):
        # Pull the next frame
        if self.nextFrame():
            return self.getGeometry()
        else:
            return None, None, None

    def getGeometry(self):
        # Get geometry
        vertex_path = os.path.join(self.path, 'vertices/' + str(self.frame.id) + '.npy')
        simplex_path = os.path.join(self.path, 'faces/' + str(self.frame.id) + '.npy')
        vertices = np.load(vertex_path)
        vertices[:,:2] += self.frame.origin.location[:2].squeeze()
        faces = np.load(simplex_path)

        # Get classes
        if self.classes:
            classes_path = os.path.join(self.path, 'classes/' + str(self.frame.id) + '.npy')
            classes = np.load(classes_path)
        else:
            confidence_path = os.path.join(self.path, 'confidence/' + str(self.frame.id) + '.npz')
            classes = np.load(confidence_path)['arr_0']

        # return data
        return vertices, faces, classes

    def seek(self, sec):
        # seek to second
        try:
            comp = rospy.Duration.from_sec(sec)
            if self.frame is None:
                print(self.nextFrame())
            while self.frame.timestamp < comp:
                if self.nextFrame() == False:
                    break
        except:
            pass
        return self.getGeometry()

    def getCurrentOrigin(self):
        if self.frame is None:
            return None
        return self.frame.origin
    
    def getCurrentTimestamp(self):
        if self.frame is None:
            return None
        return self.frame.timestamp

plotting = PlaybackLoader(path)

# Helper classes to slightly speed up rospy
class Point():
    __slots__ = __fields__ = 'x', 'y', 'z'
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

class MeshTriangleIndices():
    __slots__ = __fields__ = 'vertex_indices'
    def __init__(self, vertex_indices=np.zeros(3)):
        self.vertex_indices = vertex_indices

vertex = simplex = classes = None
# while True:
# 	vertex, simplex, classes = plotting.nextSetOfData()
# 	if vertex is not None and simplex is not None:
# 		break

vertex, simplex, classes = plotting.seek(seek_ahead)

rospy.init_node('sel_map_mesh')
mesh_uuid = str(uuid.uuid4())
mesh_topic = 'mesh'
material_service = 'get_materials'

meshMsg = MeshGeometryStamped()
meshMsg.uuid = mesh_uuid
meshMsg.header = Header(frame_id='sel_map',stamp=rospy.Time.now())
        
heightCosts = MeshVertexCostsStamped()
heightCosts.header = meshMsg.header
heightCosts.uuid = mesh_uuid
heightCosts.type = 'height'
heightCosts.mesh_vertex_costs = MeshVertexCosts(costs=vertex[:,2])


pub_tf = tf2_ros.StaticTransformBroadcaster()
map_tf_msg = TransformStamped()
world_base = rospy.get_param("world_base", "odom")
map_tf_msg.header.frame_id = world_base
map_tf_msg.child_frame_id = "sel_map"
map_tf_msg.transform.rotation.x = 0
map_tf_msg.transform.rotation.y = 0
map_tf_msg.transform.rotation.z = 0
map_tf_msg.transform.rotation.w = 1
map_tf_msg.header.stamp = rospy.Time.now()
map_tf_msg.transform.translation.x = 0
map_tf_msg.transform.translation.y = 0
map_tf_msg.transform.translation.z = 0
pub_tf.sendTransform(map_tf_msg)

# Some extra arguments
enableMat = rospy.get_param("enable_mat_display", True)
import rospkg
rospack = rospkg.RosPack()
path = rospack.get_path('sel_map')
property_file = rospy.get_param("terrain_properties", os.path.join(path, "config/terrain_properties/csail_semseg_properties.yaml"))

# Preprocess material colors
materials = MeshMaterials()
# property_file = "config/csail_semseg_properties.yaml"
# property_file = "config/fastSCNN_properties_citys.yaml"
with open(property_file) as file:
    prop_dict = yaml.load(file, Loader=yaml.FullLoader)
    keys_list = list(prop_dict)

# Load the colorscale
colorscale_args = rospy.get_param("colorscale", None)
if colorscale_args is None:
    with open(os.path.join(path, "config/colorscales/default.yaml")) as file:
        colorscale_args = yaml.load(file, Loader=yaml.FullLoader)['colorscale']
colorscale = ColorScale(args=colorscale_args)
if colorscale.bypass:
    # we're not using the colorscale here, so use properties instead
    _materials = [ColorRGBA(float(prop_dict[key]['color']['R'])/255.0,
                        float(prop_dict[key]['color']['G'])/255.0,
                        float(prop_dict[key]['color']['B'])/255.0,1) for key in keys_list]
else:
    # Map the friction to the colors
    _materials = [ColorRGBA(*colorscale.mapToColor(prop_dict[key]['friction']['mean']),1) for key in keys_list]

materials.materials = [MeshMaterial(color=_materials[i], has_texture=False) for i in range(len(_materials))]
materials.cluster_materials = np.array(range(len(_materials)))

lock = threading.Lock()

def materialCallback(req):
    if (req.uuid != mesh_uuid):
        return
    msg = MeshMaterialsStamped()
    msg.uuid = mesh_uuid
    msg.header = Header(frame_id='sel_map',stamp=rospy.Time.now())
    if (lock.acquire(False)):
        msg.mesh_materials = materials
        try:
            return GetMaterialsResponse(msg)
        finally:
            lock.release()
    elif synced_playback is None:
        with lock:
            msg.mesh_materials = materials
            return GetMaterialsResponse(msg)

pub_mesh = rospy.Publisher(mesh_topic, MeshGeometryStamped, queue_size=10)
pub_costs = rospy.Publisher(mesh_topic + '/costs', MeshVertexCostsStamped, queue_size=10)
# ROS service
if enableMat:
    mesh_mat_service = rospy.Service(material_service, GetMaterials, materialCallback)

def publishSaveMap():
    global vertex, classes

    # # Publish the transform frame
    # map_tf_msg.header.stamp = rospy.Time.now()
    # map_tf_msg.transform.translation.x = plotting.frame.origin.location[0]
    # map_tf_msg.transform.translation.y = plotting.frame.origin.location[1]
    # map_tf_msg.transform.translation.z = plotting.frame.origin.location[2]
    # pub_tf.sendTransform(map_tf_msg)

    with lock:
        # First prepare the new colors
        if plotting.classes:
            simplex_colors = classes
        else:
            simplex_colors = np.argmax(classes, axis=1)

        # not actually clustered, just for viz
        clustered_faces = [MeshFaceCluster(face_indices=np.argwhere(simplex_colors==i).flatten().astype(np.uint32)) for i in materials.cluster_materials]
        # update the message for the service
    
        # Process the message
        vertices = vertex[:,:3]
        meshMsg.mesh_geometry.vertices = [Point(*vert) for vert in vertices]
        meshMsg.mesh_geometry.faces = [MeshTriangleIndices(idxs) for idxs in simplex]
        meshMsg.header.stamp = rospy.Time.now()
        # Publish
        pub_mesh.publish(meshMsg)
        materials.clusters = clustered_faces
    # Get costs
    cost = vertices[:,2]-np.median(vertices[:,2])
    q25 = np.median(cost[cost<0])
    q75 = np.median(cost[cost>0])
    iqr = q75-q25
    cost[cost<q25-iqr*1.5] = q25-iqr*1.5
    cost[cost>q75+iqr*3] = q75+iqr*3
    heightCosts.mesh_vertex_costs = MeshVertexCosts(costs=cost)
    heightCosts.header.stamp = rospy.Time.now()
    pub_costs.publish(heightCosts)

publishSaveMap()
if synced_playback is not None:
    camera_info = rospy.wait_for_message(synced_playback, rospy.AnyMsg)
    start = rospy.Time.now()

timer = rospy.Rate(rate)
# Get initial data
vertex, simplex, classes = plotting.nextSetOfData()
timestamp = plotting.getCurrentTimestamp()
origin = plotting.getCurrentOrigin()

# import tf2_ros
# pub_tf = tf2_ros.TransformBroadcaster()
# map_tf_msg = TransformStamped()
# map_tf_msg.header.frame_id = "map"
# map_tf_msg.child_frame_id = "sel_map"
# map_tf_msg.transform.rotation.x = 0
# map_tf_msg.transform.rotation.y = 0
# map_tf_msg.transform.rotation.z = 0
# map_tf_msg.transform.rotation.w = 1

# Map transform
# map_tf_msg.transform.translation.x = origin.location[0]
# map_tf_msg.transform.translation.y = origin.location[1]
# map_tf_msg.transform.translation.z = origin.location[2]
while True:
    print(rospy.Time.now())
    print(timestamp)
    if synced_playback is not None and (start - rospy.Duration.from_sec(seek_ahead) + timestamp/playback_speed) >= rospy.Time.now():
        timer.sleep()
        # map_tf_msg.transform.translation.x = origin.location[0]
        # map_tf_msg.transform.translation.y = origin.location[1]
        # map_tf_msg.transform.translation.z = origin.location[2]
        # map_tf_msg.header.stamp = rospy.Time(0)# - self.time_delay
        # pub_tf.sendTransform(map_tf_msg)
        continue

    publishSaveMap()
    vertex, simplex, classes = plotting.nextSetOfData()
    timestamp = plotting.getCurrentTimestamp()
    origin = plotting.getCurrentOrigin()
    print(origin.location)
    if vertex is None:
        break
    timer.sleep()


print("Spinning service until ctrl+c")
rospy.spin()