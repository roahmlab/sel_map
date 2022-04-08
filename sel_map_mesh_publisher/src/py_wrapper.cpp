//TODO : figure out appropriate name
#include "TriangularMeshPublisher.hpp"
#include "TriangularMeshDataAdaptor.hpp"
#include <sel_map_mesh/TriangularMesh.hpp>
#include <sel_map_mesh/Defs.hpp>
#include <string>
#include <Eigen/Dense>

extern "C"
{
    // Define structures to simplify function definitions
    typedef struct {
        void* data;
        ssize_t rows;
        ssize_t cols;
    } _sel_map_matrix;

    typedef struct {
        void** start;
        ssize_t size;
        ssize_t el_size;
    } _sel_map_confidence_list;

    typedef struct {
        char** argv;
        int argc;
    } _sel_map_sys_argv;

    // Create and return a pointer to an instance of TriangularMeshPublisher
    void* _sel_map_CreateTriangularMeshPublisherInstance(void* mesh, char* uuid, char* frame_id, char* mesh_topic);
    
    // Delete the TriangularMeshPublisher instance provided via pointer
    void _sel_map_DeleteTriangularMeshPublisherInstance(void* self);

    // Check if the TriangularMeshPublisher is alive
    char _sel_map_TriangularMeshPublisher_pubAlive(void* self);

    // Do a publish direct from the current data cache (whether that is the map or the last data loaded)
    void _sel_map_TriangularMeshPublisher_publishMeshDirect(void* self);
    
    // Do a publish based on the data provided
    void _sel_map_TriangularMeshPublisher_publishMeshData(void* self, _sel_map_matrix* vertexVector, _sel_map_matrix* simplices);

    // Get the one hot classification based on the current data cache (whether that is the map or the last data loaded)
    unsigned int _sel_map_TriangularMeshPublisher_getSingleClassificationsDirect(void* self, _sel_map_matrix* classifications_out);

    // Get the one hot classification based on the data provided
    unsigned int _sel_map_TriangularMeshPublisher_getSingleClassificationsData(void* self, _sel_map_confidence_list* classifications, _sel_map_matrix* classifications_out);

    // A wrapped method of roscpp_init
    void _sel_map_roscpp_init(char* node_name, _sel_map_sys_argv* argv);

    // A wrapped method of roscpp_shutdown
    void _sel_map_roscpp_shutdown();

}

typedef sel_map::mesh::TriangularMesh MeshType;
typedef sel_map::publisher::TriangularMeshDataAdaptor DataAdaptor;
typedef sel_map::publisher::TriangularMeshPublisher<DataAdaptor> MeshPublisher;

// convert to typedef'd struct as per more standard styles later
inline MeshType* getMeshRef(void* self){
    MeshType* ref = reinterpret_cast<MeshType*>(self);
    return ref;
}

// convert to typedef'd struct as per more standard styles later
inline MeshPublisher* getRef(void* self){
    MeshPublisher* ref = reinterpret_cast<MeshPublisher*>(self);
    return ref;
}

// Create
void* _sel_map_CreateTriangularMeshPublisherInstance(void* mesh, char* uuid, char* frame_id, char* mesh_topic)
{
    auto mesh_ref = getMeshRef(mesh);
    return new
        MeshPublisher(DataAdaptor(*mesh_ref), std::string(uuid), std::string(frame_id), std::string(mesh_topic));
}

// Delete
void _sel_map_DeleteTriangularMeshPublisherInstance(void* self)
{
    auto ref = getRef(self);
    delete ref; 
}

// Check if alive
char _sel_map_TriangularMeshPublisher_pubAlive(void* self)
{
    auto ref = getRef(self);
    return ref->pubAlive();
}

// Direct publish
void _sel_map_TriangularMeshPublisher_publishMeshDirect(void* self)
{
    auto ref = getRef(self);
    ref->mesh_adaptor.update_from_mesh();
    ref->publishMesh();
}

// Publish from data
void _sel_map_TriangularMeshPublisher_publishMeshData(void* self, _sel_map_matrix* _vertexVector, _sel_map_matrix* _simplices)
{
    auto ref = getRef(self);
    // translate the arguments into Eigen
    using sel_map::mesh::PointWithCovArray_t;
    using sel_map::mesh::PointArray_t;
    using sel_map::mesh::IndexArray_t;
    
    // Validate types
    assert(_vertexVector->cols == 4 || _vertexVector->rows != 0);
    auto vertexVector = Eigen::Map<const PointWithCovArray_t, Eigen::Aligned16>((const PointWithCovArray_t::Scalar*)_vertexVector->data, _vertexVector->rows, _vertexVector->cols);
    
    // assert(_vertexNormals->cols == 3 || _vertexNormals->rows != 0);
    // auto vertexNormals = Eigen::Map<const PointArray_t, Eigen::Aligned16>((const PointArray_t::Scalar*)_vertexNormals->data, _vertexNormals->rows, _vertexNormals->cols);
    
    assert(_simplices->cols == 3 || _simplices->rows != 0);
    auto simplices = Eigen::Map<const IndexArray_t, Eigen::Aligned16>((const IndexArray_t::Scalar*)_simplices->data, _simplices->rows, _simplices->cols);

    // Update and publish
    ref->mesh_adaptor.update_vertices(vertexVector);
    // ref->mesh_adaptor.update_vertex_normals(vertexNormals);
    ref->mesh_adaptor.update_faces(simplices);

    ref->publishMesh();
}

// Get direct classifications
unsigned int _sel_map_TriangularMeshPublisher_getSingleClassificationsDirect(void* self, _sel_map_matrix* classifications_out)
{
    auto ref = getRef(self);
    assert(classifications_out->cols == 1 || classifications_out->rows != 0);
    ref->mesh_adaptor.update_from_mesh();
    return ref->getSingleClassifications((int*)classifications_out->data, classifications_out->rows);
}

// Get classifications from data
unsigned int _sel_map_TriangularMeshPublisher_getSingleClassificationsData(void* self, _sel_map_confidence_list* classifications, _sel_map_matrix* classifications_out)
{
    auto ref = getRef(self);

    // validate and update
    // assert(classifications->size == ref->mesh_adaptor.num_of_classifications);
    assert(classifications->el_size == ref->mesh_adaptor.num_of_classes);
    ref->mesh_adaptor.update_classifications((double**)classifications->start, classifications->size);

    // Get the result
    assert(classifications_out->cols == 1 || classifications_out->rows != 0);
    return ref->getSingleClassifications((int*)classifications_out->data, classifications_out->rows);
}

// To retain roscpp, keep a nodehandle in static var for this function
void _sel_map_roscpp_init_shutdown(bool init, char* node_name, _sel_map_sys_argv* argv)
{
    static ros::NodeHandle* keepAlive = nullptr;

    if (init){
        // Start the rosnode without overriding the SIGINT handler (so rospy still works as intended)
        // and give this an anonymous name in case we have node name collisions (unlikely to happen)
        ros::init(argv->argc, argv->argv, std::string(node_name), ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        keepAlive = new ros::NodeHandle();
    }
    else {
        delete keepAlive;
        keepAlive = nullptr;
        ros::shutdown();
    }
}

// A wrapped method of roscpp_init
void _sel_map_roscpp_init(char* node_name, _sel_map_sys_argv* argv)
{
    // Do nothing if roscpp is already initialized
    if (ros::isInitialized()) return;

    // Otherwise use the above helper to start it
    _sel_map_roscpp_init_shutdown(true, node_name, argv);
}


void _sel_map_roscpp_shutdown()
{
    // Do nothing if roscpp is dead
    // if (!ros::isInitialized()) return;

    // Otherwise use the above helper to kill it
    _sel_map_roscpp_init_shutdown(false, nullptr, nullptr);
}
