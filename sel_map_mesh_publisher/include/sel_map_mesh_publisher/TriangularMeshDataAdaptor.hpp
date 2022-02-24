#pragma once

#include <tuple>

#include <sel_map_mesh/Defs.hpp>
#include <sel_map_mesh/TriangularMesh.hpp>

// Placing all lib elements into a sel_map namespace, partly in case this is extended upon later, partly to reduce pollution.
namespace sel_map::publisher{

    // This is very ugly
    class TriangularMeshDataAdaptor {
        typedef double classifications_t;
        typedef unsigned int num_classifications_t;
        typedef unsigned int num_classes_t;

        sel_map::mesh::TriangularMesh* mesh;

        sel_map::mesh::PointWithCovArray_t temporaryVertexVector;
        sel_map::mesh::PointArray_t temporaryVertexNormals;
        sel_map::mesh::IndexArray_t temporarySimplices;
        std::vector<double*> temporaryClassifications;
        // std::vector<std::vector<double>> temporaryClassificationsData;

        Eigen::Ref<const sel_map::mesh::PointWithCovArray_t, Eigen::Aligned16> vertexVector;
        Eigen::Ref<const sel_map::mesh::PointArray_t, Eigen::Aligned16> vertexNormals;
        Eigen::Ref<const sel_map::mesh::IndexArray_t, Eigen::Aligned16> simplices;
        classifications_t** classifications;

        public:
        num_classifications_t num_of_classifications;
        num_classes_t num_of_classes;
        TriangularMeshDataAdaptor(const Eigen::Ref<const sel_map::mesh::PointWithCovArray_t, Eigen::Aligned16>& vertexVector,
                                //   const Eigen::Ref<const sel_map::mesh::PointArray_t, Eigen::Aligned16>& vertexNormals,
                                  const Eigen::Ref<const sel_map::mesh::IndexArray_t, Eigen::Aligned16>& simplices,
                                  classifications_t** classifications, num_classes_t num_of_classes);

        TriangularMeshDataAdaptor(sel_map::mesh::TriangularMesh& mesh);
        
        void change_mesh(sel_map::mesh::TriangularMesh& mesh);

        void update_from_mesh();

        void update_vertices(const Eigen::Ref<const sel_map::mesh::PointWithCovArray_t, Eigen::Aligned16>& new_vertexVector);
        // void update_vertex_normals(const Eigen::Ref<const sel_map::mesh::PointArray_t, Eigen::Aligned16>& new_vertexNormals);
        void update_faces(const Eigen::Ref<const sel_map::mesh::IndexArray_t, Eigen::Aligned16>& new_simplices);
        void update_classifications(classifications_t** new_classifications, num_classifications_t size);

        Eigen::Ref<const sel_map::mesh::PointWithCovArray_t, Eigen::Aligned16>* get_vertices();
        Eigen::Ref<const sel_map::mesh::PointArray_t, Eigen::Aligned16>* get_vertex_normals();
        Eigen::Ref<const sel_map::mesh::IndexArray_t, Eigen::Aligned16>* get_faces();
        std::tuple<classifications_t**, num_classifications_t, num_classes_t> get_classifications();
    };
    
}
