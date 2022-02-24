#include "TriangularMeshDataAdaptor.hpp"

#include <sel_map_mesh/Defs.hpp>
#include <sel_map_mesh/TriangularMesh.hpp>

// Avoid having to rewrite the object repeatedly
using sel_map::publisher::TriangularMeshDataAdaptor;
using sel_map::mesh::PointWithCovArray_t;
using sel_map::mesh::PointArray_t;
using sel_map::mesh::IndexArray_t;
using sel_map::mesh::TriangularMesh;

// Constructor from data alone
TriangularMeshDataAdaptor::
TriangularMeshDataAdaptor(const Eigen::Ref<const PointWithCovArray_t, Eigen::Aligned16>& vertexVector,
                        // const Eigen::Ref<const PointArray_t, Eigen::Aligned16>& vertexNormals,
                        const Eigen::Ref<const IndexArray_t, Eigen::Aligned16>& simplices,
                        classifications_t** classifications, num_classes_t num_of_classes)
                        : temporaryVertexVector(),
                          temporaryVertexNormals(),
                          temporarySimplices(),
                          temporaryClassifications(),
                          vertexVector(vertexVector),
                          vertexNormals(temporaryVertexNormals),
                          simplices(simplices),
                          classifications(classifications),
                          num_of_classes(num_of_classes)
{
    num_of_classifications = simplices.rows();
};

// Constructor from existing mesh
TriangularMeshDataAdaptor::TriangularMeshDataAdaptor(TriangularMesh& mesh)
            : temporaryVertexVector(),
              temporaryVertexNormals(),
              temporarySimplices(),
              temporaryClassifications(),
              vertexVector(temporaryVertexVector),
              vertexNormals(temporaryVertexNormals),
              simplices(temporarySimplices)
{
    change_mesh(mesh);
};

void TriangularMeshDataAdaptor::change_mesh(TriangularMesh& mesh){
    this->mesh = &mesh;
    num_of_classes = mesh.terrainClasses;
    update_from_mesh();
};

void TriangularMeshDataAdaptor::update_from_mesh(){
    std::tie(temporaryVertexVector, temporarySimplices, temporaryClassifications) = mesh->getTrimmedMesh();
    update_vertices(temporaryVertexVector);
    update_faces(temporarySimplices);
    // for(auto& el : temporaryClassifications){
    //     temporaryClassificationsData.push_back(std::vector<double>(el, el+num_of_classes));
    //     el = temporaryClassificationsData.back().data();
    // }
    update_classifications(temporaryClassifications.data(), temporaryClassifications.size());
};

// Utility functions to update the reference as needed (slightly hacky)
void TriangularMeshDataAdaptor::update_vertices(const Eigen::Ref<const PointWithCovArray_t, Eigen::Aligned16>& new_vertexVector)
{
    // Destroy the Ref and construct it with the new data
    vertexVector.~Ref();
    new(&vertexVector) Eigen::Ref<const PointWithCovArray_t, Eigen::Aligned16>(new_vertexVector);
};

// void TriangularMeshDataAdaptor::update_vertex_normals(const Eigen::Ref<const PointArray_t, Eigen::Aligned16>& new_vertexNormals)
// {
//     // Destroy the Ref and construct it with the new data
//     vertexNormals.~Ref();
//     new(&vertexNormals) Eigen::Ref<const PointArray_t, Eigen::Aligned16>(new_vertexNormals);
// };

void TriangularMeshDataAdaptor::update_faces(const Eigen::Ref<const IndexArray_t, Eigen::Aligned16>& new_simplices)
{
    // Destroy the Ref and construct it with the new data
    simplices.~Ref();
    new(&simplices) Eigen::Ref<const IndexArray_t, Eigen::Aligned16>(new_simplices);
};

void TriangularMeshDataAdaptor::update_classifications(classifications_t** new_classifications, num_classifications_t size)
{
    classifications = new_classifications;
    num_of_classifications = size;
}


// Utility functions to get pointers to the stored data
Eigen::Ref<const sel_map::mesh::PointWithCovArray_t, Eigen::Aligned16>*
TriangularMeshDataAdaptor::get_vertices()
{
    return &vertexVector;
};

Eigen::Ref<const sel_map::mesh::PointArray_t, Eigen::Aligned16>*
TriangularMeshDataAdaptor::get_vertex_normals()
{
    return &vertexNormals;
};

Eigen::Ref<const sel_map::mesh::IndexArray_t, Eigen::Aligned16>*
TriangularMeshDataAdaptor::get_faces()
{
    return &simplices;
};

std::tuple<TriangularMeshDataAdaptor::classifications_t**,
           TriangularMeshDataAdaptor::num_classifications_t,
           TriangularMeshDataAdaptor::num_classes_t>
TriangularMeshDataAdaptor::get_classifications()
{
    return std::tuple<classifications_t**, num_classifications_t, num_classes_t>
            (classifications, num_of_classifications, num_of_classes);
};
