#include "TriangularMeshPublisher.hpp"

#include <tuple>
#include <algorithm>
#include <iterator>

#include <sel_map_mesh/Defs.hpp>
#include <sel_map_mesh/TriangularMesh.hpp>
// #include <boost/uuid/uuid.hpp>
// #include <boost/uuid/uuid_generators.hpp>

#include "TriangularMeshDataAdaptor.hpp"

// Avoid having to rewrite the object repeatedly
using sel_map::publisher::TriangularMeshPublisher;
using sel_map::msg_adaptor::MeshGeometryStampedCustom;
using sel_map::mesh::PointWithCovArray_t;
using sel_map::mesh::PointArray_t;
using sel_map::mesh::IndexArray_t;

// Constructor from data alone
template <typename MeshAdaptorType>
TriangularMeshPublisher<MeshAdaptorType>::
TriangularMeshPublisher(const MeshAdaptorType& mesh_adaptor, std::string uuid, std::string frame_id, std::string mesh_topic)
    : node_handle(), mesh_adaptor(mesh_adaptor)
{
    cached_message.uuid = uuid;
    cached_message.header.frame_id = frame_id;
    // Setup a publisher with a queue size of 2 and latching enabled.
    mesh_publisher = node_handle.advertise<MeshGeometryStampedCustom>(mesh_topic, 2, true);

    // Publish an initial message
    publishMesh();
};

// Make sure we can check if the publisher actually survived or not.
template <typename MeshAdaptorType>
bool TriangularMeshPublisher<MeshAdaptorType>::pubAlive()
{
    return mesh_publisher;
};

// Update cached message and publish
template <typename MeshAdaptorType>
void TriangularMeshPublisher<MeshAdaptorType>::publishMesh()
{
    // Update time
    cached_message.header.stamp = ros::Time::now();

    // Make sure the mesh geometry is up to date
    cached_message.mesh_geometry.vertices.vertices_ptr              = mesh_adaptor.get_vertices();
    cached_message.mesh_geometry.vertex_normals.vertex_normals_ptr  = mesh_adaptor.get_vertex_normals();
    cached_message.mesh_geometry.faces.faces_ptr                    = mesh_adaptor.get_faces();

    // Publish it only if we have any vertices
    if (cached_message.mesh_geometry.vertices.vertices_ptr->rows()) 
        mesh_publisher.publish(cached_message);
};

// Get the one hot classification values
// NOT ALWAYS A SAFE OPERATION**
template <typename MeshAdaptorType>
unsigned int TriangularMeshPublisher<MeshAdaptorType>::getSingleClassifications(int* buffer, unsigned int length)
{
    // Get the latest from the MeshAdaptor
    // This is where it could be made safer!
    double** classifications;
    unsigned int num_of_classifications;
    unsigned int num_of_classes;
    std::tie(classifications, num_of_classifications, num_of_classes) = mesh_adaptor.get_classifications();
    ROS_ASSERT_MSG(length >= num_of_classifications, "You must allocate a large enough buffer for single classifications!");

    // Iterate through and isolate the highest value class for every classification
    for(unsigned int i = 0; i < num_of_classifications; ++i){
        double* ptr_to_max = std::max_element(classifications[i], (classifications[i])+num_of_classes);
        buffer[i] = std::distance(classifications[i], ptr_to_max);
    }
    // Return the num of classifications in case a resize is desired
    return num_of_classifications;
}

// Forward declare Publisher with the DataAdaptor
template class TriangularMeshPublisher<sel_map::publisher::TriangularMeshDataAdaptor>;