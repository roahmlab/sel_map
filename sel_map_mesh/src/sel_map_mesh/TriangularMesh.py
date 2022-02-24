import numpy as np
import ctypes
import os
# import time

lib = ctypes.CDLL('libsel_map_mesh.so')

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


class TriangularMesh():
    def __init__(self, bounds, originHeight=0, elementLength=1,
				 pointLimit=20, heightPartition=3, heightSafetyCheck=0.5,
				 terrainClasses=150, groundtruth=False, verbose=False):

        f_obj = lib._sel_map_CreateTriangularMeshInstance
        f_obj.argtypes = [np.ctypeslib.ndpointer(dtype=ctypes.c_double,shape=(3,)),
                        ctypes.c_double, ctypes.c_double,
                        ctypes.c_uint, ctypes.c_float, ctypes.c_double, ctypes.c_uint]
        f_obj.restype = ctypes.c_void_p
        # double* bounds, double* origin, double elementLength,
        # unsigned int pointLimit, float heightPartition, double heightSafetyCheck,
        # unsigned int terrainClasses
        self.obj = f_obj(np.array(bounds, dtype=ctypes.c_double), originHeight, elementLength, pointLimit,
                        heightPartition, heightSafetyCheck, terrainClasses)

        self.originHeight = originHeight
        self.bounds = bounds
        self.pointLimit = pointLimit
        self.elementLength = elementLength
        self.heightPartition = heightPartition
        self.heightSafetyCheck = heightSafetyCheck
        self.terrainClasses = terrainClasses

        self.width = round(bounds[0]*2 / elementLength)+1

        self.verbose = verbose
        self.groundtruth = groundtruth

        # Eulers formula from meshGenerator
        E = 3*(self.width*self.width) - 4*self.width + 1
        V = self.width*self.width
        self.num_elements = E - V + 1

        # self.elements = None
        
        # Preallocate the space for the trimmed mesh info
        # TODO: Devise a better way to handle this. This is the most memory intensive part!
        self.vertexBuffer = _sel_map_matrix()
        self.vertexBuffer.from_ndarray(np.zeros([self.width*self.width,4]))
        self.simplexBuffer = _sel_map_matrix()
        self.simplexBuffer.from_ndarray(np.zeros([self.num_elements,3]), dtype=ctypes.c_uint32)
        self.confidenceBuffer = _sel_map_confidence_list()
        self.confidenceBuffer.from_list_of_ndarray([np.zeros([self.terrainClasses]) for _ in range(self.num_elements)])

    def __del__(self):
        f_del = lib._sel_map_DeleteTriangularMeshInstance
        f_del.argtypes = [ctypes.c_void_p]
        f_del(self.obj)
    
    def create(self):
        # create the mesh
        f_create = lib._sel_map_TriangularMesh_create
        f_create.argtypes = [ctypes.c_void_p]
        f_create(self.obj)

    def advance(self, points = None, classpoints = None, z_height=None):
        # advance the mesh with the given points
        f_advance = lib._sel_map_TriangularMesh_advance
        f_advance.argtypes = [ctypes.c_void_p, ctypes.POINTER(_sel_map_matrix), ctypes.POINTER(_sel_map_matrix), ctypes.c_double]
        #void* ptr, _sel_map_matrix* _points, _sel_map_matrix* _classpoints
        # Fill in stuff for function call
        points_s = _sel_map_matrix()
        points_s.from_ndarray(points)
        classpoints_s = _sel_map_matrix()
        classpoints_s.from_ndarray(classpoints)
        if z_height is None:
            z_height = self.originHeight
        else:
            self.originHeight = z_height
        # advance the mesh
        f_advance(self.obj, ctypes.byref(points_s), ctypes.byref(classpoints_s), z_height)

    def shiftMeshElements(self, x_elem_shift = 0, y_elem_shift = 0):
        # shift the mesh by n elements in each specified direction
        f_shift = lib._sel_map_TriangularMesh_shiftMeshElements
        f_shift.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
        f_shift(self.obj, int(x_elem_shift), int(y_elem_shift))
    
    def saveMesh(self, output_dir, frame):
        # saves the vertexvector, simplices, and classes
        # calls getMesh to get the data then saves it as specified
        vertex_file = os.path.join(output_dir,'vertex_' + str(frame) + '.npy')
        normals_file = os.path.join(output_dir,'normals_' + str(frame) + '.npy')
        simplex_file = os.path.join(output_dir,'simplex_' + str(frame) + '.npy')
        class_infer_file = os.path.join(output_dir,'class_infer_' + str(frame) + '.npz')

        # Get the mesh stuff
        vertexVector, vertexNormals, simplices = self.getMesh()
        class_infer = self.getClasses()

        # save it
        np.save(vertex_file, vertexVector)
        np.save(normals_file, vertexNormals)
        np.save(simplex_file, simplices)
        np.savez(class_infer_file, class_infer)

    def _getMeshRaw(self):
        # returns the vertexvector, vertexnormals, and simplices
        f_getMesh = lib._sel_map_TriangularMesh_getMesh
        f_getMesh.argtypes = [ctypes.c_void_p, ctypes.POINTER(_sel_map_matrix), ctypes.POINTER(_sel_map_matrix),
                            ctypes.POINTER(_sel_map_matrix)]
        # Create the storage units in python so we don't lose scope
        vertexVector_s = _sel_map_matrix()
        vertexNormals_s = _sel_map_matrix()
        simplices_s = _sel_map_matrix()
        # Call the functions with the pointers
        f_getMesh(self.obj, ctypes.byref(vertexVector_s),
                    ctypes.byref(vertexNormals_s), ctypes.byref(simplices_s))
        # passthrough
        return vertexVector_s, vertexNormals_s, simplices_s

    def getMesh(self, vertexVector_s=None, vertexNormals_s=None, simplices_s=None):
        # returns the vertexvector, vertexnormals, and simplices
        if vertexVector_s is None or vertexNormals_s is None or simplices_s is None:
            vertexVector_s, vertexNormals_s, simplices_s = self._getMeshRaw()
        # Buffer into python, then load into numpy, memory remains handled by sel_map
        vertexVector = vertexVector_s.as_ndarray()
        vertexNormals = vertexNormals_s.as_ndarray()
        simplices = simplices_s.as_ndarray(ctypes.c_uint32)
        return vertexVector, vertexNormals, simplices

    def _getClassesRaw(self):
        # returns the classes
        f_getClasses = lib._sel_map_TriangularMesh_getClasses
        f_getClasses.argtypes = [ctypes.c_void_p, ctypes.POINTER(_sel_map_confidence_list)]
        # Create the storage units in python so we don't lose scope
        classes_s = _sel_map_confidence_list()
        # Call the functions with the pointers
        f_getClasses(self.obj, ctypes.byref(classes_s))
        # passthrough
        return classes_s

    def getClasses(self, classes_s=None):
        # returns the classes
        if classes_s is None:
            classes_s = self._getClassesRaw()
        # Buffer into python, then load into numpy, memory remains handled by sel_map
        classes = classes_s.as_list_of_ndarray()
        return classes

    def getTrimmed(self):
        # returns the vertexvector, simplices, and confidence matrices
        f_getMesh = lib._sel_map_TriangularMesh_getTrimmed
        f_getMesh.argtypes = [ctypes.c_void_p, ctypes.POINTER(_sel_map_matrix), ctypes.POINTER(_sel_map_matrix),
                            ctypes.POINTER(_sel_map_confidence_list)]
        # Call the functions with the pointers
        f_getMesh(self.obj, ctypes.byref(self.vertexBuffer),
                    ctypes.byref(self.simplexBuffer), ctypes.byref(self.confidenceBuffer))
        
        # Copy from the buffers
        vertexVector = self.vertexBuffer.c_arr[:self.vertexBuffer.rows,:4]
        simplices = self.simplexBuffer.c_arr[:self.simplexBuffer.rows,:3]
        classes = self.confidenceBuffer.c_list[:self.confidenceBuffer.size]

        # Reset the buffer sizes
        self.vertexBuffer.rows = self.width*self.width
        self.simplexBuffer.rows = self.num_elements
        self.confidenceBuffer.size = self.num_elements

        return vertexVector, simplices, classes

    def clean(self, lazy=False):
        # calls the clean function on the internal mesh
        f_clean = lib._sel_map_TriangularMesh_clean
        f_clean.argtypes = [ctypes.c_void_p, ctypes.c_uint8]
        f_clean(self.obj, ctypes.c_uint8(lazy))

