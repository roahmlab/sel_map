from ast import arg
from time import sleep
import numpy as np
import ctypes
import sys
from sel_map_mesh import TriangularMesh
# import time

lib = ctypes.CDLL('libsel_map_mesh_publisher.so')

# https://stackoverflow.com/questions/9895787/memory-alignment-for-fast-fft-in-python-using-shared-arrays
def aligned(a, alignment = 16):
    if (a.ctypes.data % alignment) == 0:
        return a
    print("realignment needed")
    assert alignment % a.itemsize == 0
    extra = alignment // a.itemsize
    buf = np.empty(a.size + extra, dtype = a.dtype)
    ofs = (-buf.ctypes.data % alignment) // a.itemsize
    aa = buf[ofs:ofs + a.size].reshape(a.shape)
    np.copyto(aa, a)
    assert aa.ctypes.data % alignment == 0
    return aa

class _sel_map_matrix(ctypes.Structure):
    _fields_ = [
        ("ptr", ctypes.c_void_p),
        ("rows", ctypes.c_ssize_t),
        ("cols", ctypes.c_ssize_t)]
    
    def as_ndarray(self, dtype=ctypes.c_double):
        if self.ptr is None:
            return np.ndarray((0,0))
        casted_ptr = ctypes.cast(self.ptr, ctypes.POINTER(dtype))
        arr = np.ctypeslib.as_array(casted_ptr, shape=(self.rows, self.cols))
        return arr
    
    def from_ndarray(self, arr, dtype=ctypes.c_double):
        if arr is None:
            self.ptr = None
            self.rows = 0
            self.cols = 0
            return
        # Store it so that it retains refcount until this goes out of scope!
        self.c_arr = np.ascontiguousarray(arr, dtype=dtype)
        self.c_arr = aligned(self.c_arr)
        self.ptr = self.c_arr.ctypes.data_as(ctypes.c_void_p)
        self.rows = self.c_arr.shape[0]
        self.cols = self.c_arr.shape[1]


class _sel_map_confidence_list(ctypes.Structure):
    _fields_ = [
        ("first", ctypes.POINTER(ctypes.c_void_p)),
        ("size", ctypes.c_ssize_t),
        ("el_size", ctypes.c_ssize_t)]
    
    def as_list_of_pointers(self, dtype=ctypes.c_double):
        if self.first is None:
            return []
        casted_ptr = ctypes.cast(self.first, ctypes.POINTER(ctypes.POINTER(dtype)))
        return casted_ptr[:self.size]
    
    def as_list_of_lists(self, dtype=ctypes.c_double):
        ptr_list = self.as_list_of_pointers(dtype)
        return [el[:self.el_size] for el in ptr_list]
    
    def as_list_of_ndarray(self, dtype=ctypes.c_double):
        ptr_list = self.as_list_of_pointers(dtype)
        return [np.ctypeslib.as_array(el, shape=(self.el_size,)) for el in ptr_list]
    
    def from_list_of_ndarray(self, np_list, dtype=ctypes.c_double):
        self.size = len(np_list)
        self.el_size = len(np_list[0])
        self.c_list = [np.ascontiguousarray(arr, dtype=dtype) for arr in np_list]
        self.ptr_list = [c_arr.ctypes.data_as(ctypes.c_void_p) for c_arr in self.c_list]
        list_type = (ctypes.c_void_p * self.size)
        temp = list_type(*self.ptr_list)
        self.first = ctypes.cast(temp, ctypes.POINTER(ctypes.c_void_p))


class _sel_map_sys_argv(ctypes.Structure):
    _fields_ = [
        ("argv", ctypes.POINTER(ctypes.c_char_p)),
        ("argc", ctypes.c_int)]
    
    def from_argv_list(self, argv_list):
        self.argc = len(argv_list)
        ptr_list = [str.encode('utf-8') for str in argv_list]
        self.argv = (ctypes.c_char_p * len(argv_list))(*ptr_list)


def roscpp_init(node_name="MeshPublishHelper", argv_list=sys.argv):
    # Strip out special ros parameters!
    argv_list = [arg for arg in argv_list if not arg.startswith('__')]
    # initializes the roscpp node
    f_init = lib._sel_map_roscpp_init
    f_init.argtypes = [ctypes.c_char_p, ctypes.POINTER(_sel_map_sys_argv)]
    _argv = _sel_map_sys_argv()
    _argv.from_argv_list(argv_list)
    f_init(node_name.encode('utf-8'), ctypes.byref(_argv))


def roscpp_shutdown():
    # shuts down the roscpp node
    f_shutdown = lib._sel_map_roscpp_shutdown
    f_shutdown()


class TriangularMeshPublisher():
    def __init__(self, mesh, uuid, frame_id="sel_map", mesh_topic="mesh"):

        self.mesh_obj = mesh.obj
        self.num_classes = mesh.terrainClasses
        self.num_elements = mesh.num_elements

        f_obj = lib._sel_map_CreateTriangularMeshPublisherInstance
        f_obj.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]
        f_obj.restype = ctypes.c_void_p
        # double* bounds, double* origin, double elementLength,
        # unsigned int pointLimit, float heightPartition, double heightSafetyCheck,
        # unsigned int terrainClasses
        self.obj = f_obj(self.mesh_obj, uuid.encode('utf-8'), frame_id.encode('utf-8'), mesh_topic.encode('utf-8'))

        self.uuid = self.obj
        self.frame_id = frame_id
        self.mesh_topic = mesh_topic

    def __del__(self):
        f_del = lib._sel_map_DeleteTriangularMeshPublisherInstance
        f_del.argtypes = [ctypes.c_void_p]
        f_del(self.obj)
    
    def alive(self):
        # Check if the publisher is alive
        f_alive = lib._sel_map_TriangularMeshPublisher_pubAlive
        f_alive.argtypes = [ctypes.c_void_p]
        f_alive.restype = ctypes.c_byte
        alive = f_alive(self.obj)
        return alive != 0
        
    def publish(self, vertices=None, faces=None, passthrough=False):
        # If any of the vertices, normals, or faces are None, direct publish
        if vertices is None or faces is None:
            f_publish = lib._sel_map_TriangularMeshPublisher_publishMeshDirect
            f_publish.argtypes = [ctypes.c_void_p]
            f_publish(self.obj)
        else:
            f_publish = lib._sel_map_TriangularMeshPublisher_publishMeshData
            f_publish.argtypes = [ctypes.c_void_p, ctypes.POINTER(_sel_map_matrix), ctypes.POINTER(_sel_map_matrix)]
            _vertices = vertices
            # _vertex_normals = vertex_normals
            _faces = faces
            if not passthrough:
                _vertices = _sel_map_matrix()
                _vertices.from_ndarray(vertices)
                # _vertex_normals = _sel_map_matrix()
                # _vertex_normals.from_ndarray(vertex_normals)
                _faces = _sel_map_matrix()
                _faces.from_ndarray(faces, dtype=ctypes.c_uint32)
            f_publish(self.obj, ctypes.byref(_vertices), ctypes.byref(_faces))

    def get_onehot_classes(self, classifications=None, passthrough=True):
        # Prep the return storage
        res = _sel_map_matrix()
        res.from_ndarray(np.zeros((self.num_elements,1), dtype=ctypes.c_int), dtype=ctypes.c_int)
        # Get direct classes
        if classifications is None:
            f_classes = lib._sel_map_TriangularMeshPublisher_getSingleClassificationsDirect
            f_classes.argtypes = [ctypes.c_void_p, ctypes.POINTER(_sel_map_matrix)]
            f_classes.restype = ctypes.c_uint
            size = f_classes(self.obj, ctypes.byref(res))
        # Otherwise, update and get classes
        else:
            f_classes = lib._sel_map_TriangularMeshPublisher_getSingleClassificationsData
            f_classes.argtypes = [ctypes.c_void_p, ctypes.POINTER(_sel_map_confidence_list), ctypes.POINTER(_sel_map_matrix)]
            f_classes.restype = ctypes.c_uint
            _classifications = classifications
            if not passthrough:
                _classifications = _sel_map_confidence_list()
                _classifications.from_list_of_ndarray(classifications)
            size = f_classes(self.obj, ctypes.byref(_classifications), ctypes.byref(res))
        # Return the buffer from _sel_map_matrix
        return res.c_arr.flatten()[:size]
