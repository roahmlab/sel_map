//TODO : figure out appropriate name
// Build with
// g++ -I./include/ -I./include/mesh -I./libs/eigen3 -I./libs/nanoflann src/mesh/*.cpp -O3 -Ofast -frename-registers -march=native -fopenmp -D_GLIBCXX_PARALLEL -shared -Wl,-soname,sel_map_mesh -fPIC -o lsel_map_mesh.so
#include "Element.hpp"
#include "TriangleElement.hpp"
#include "Mesh.hpp"
#include "TriangularMesh.hpp"
#include <cstdarg>
#include <Eigen/Dense>
#include <iostream>

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

    // Create and return a pointer to an instance of TriangularMesh
    void* _sel_map_CreateTriangularMeshInstance(double* bounds, double origin, double elementLength,
                unsigned int pointLimit, float heightPartition, double heightSafetyCheck,
                unsigned int terrainClasses);
    
    // Delete the TriangularMesh instance provided via pointer
    void _sel_map_DeleteTriangularMeshInstance(void* self);

    // The create method of TriangularMesh
    void _sel_map_TriangularMesh_create(void* self);

    // The advance method of TriangularMesh
    void _sel_map_TriangularMesh_advance(void* self, const _sel_map_matrix* _points, const _sel_map_matrix* _classpoints, double z_height);
    
    // Shift the mesh
    void _sel_map_TriangularMesh_shiftMeshElements(void* self, int x_elem_shift, int y_elem_shift);

    // Get mesh parameters
    void _sel_map_TriangularMesh_getMesh(void* self, _sel_map_matrix* vertexVector, _sel_map_matrix* vertexNormals, _sel_map_matrix* simplices);

    // Get mesh classes
    void _sel_map_TriangularMesh_getClasses(void* self, _sel_map_confidence_list* classifications);

    // Get trimmed data
    void _sel_map_TriangularMesh_getTrimmed(void* self, _sel_map_matrix* vertexVector, _sel_map_matrix* simplices, _sel_map_confidence_list* classifications);

    // The clean method of TriangularMesh
    void _sel_map_TriangularMesh_clean(void* self, uint8_t lazy);

}


// convert to typedef'd struct as per more standard styles later
inline sel_map::mesh::TriangularMesh* getRef(void* self){
    sel_map::mesh::TriangularMesh* ref = reinterpret_cast<sel_map::mesh::TriangularMesh*>(self);
    return ref;
}

// Create
void* _sel_map_CreateTriangularMeshInstance(double* bounds, double origin, double elementLength,
            unsigned int pointLimit, float heightPartition, double heightSafetyCheck,
            unsigned int terrainClasses)
{
    return new
        sel_map::mesh::TriangularMesh(bounds, origin, elementLength,
            pointLimit, heightPartition, heightSafetyCheck, terrainClasses);
}

// Delete
void _sel_map_DeleteTriangularMeshInstance(void* self)
{
    auto ref = getRef(self);
    delete ref; 
}

// initialize
void _sel_map_TriangularMesh_create(void* self)
{
    auto ref = getRef(self);
    ref->create();
}

// advance the mesh
void _sel_map_TriangularMesh_advance(void* self, const _sel_map_matrix* _points, const _sel_map_matrix* _classpoints, double z_height)
{
    auto ref = getRef(self);
    // translate the arguments into Eigen
    using sel_map::mesh::RowArray_t;
    auto points = Eigen::Map<const RowArray_t, Eigen::Aligned16>(sel_map::mesh::NoPoints.data(), 0, 0);
    auto classpoints = points;

    if(_points && _points->data){
        assert(_points->cols == 4 || _points->rows != 0);
        points.~Map();
        new(&points) Eigen::Map<const RowArray_t, Eigen::Aligned16>((const double*)_points->data, _points->rows, _points->cols);
    }
    if(_classpoints && _classpoints->data){
        assert(_classpoints->cols == ref->terrainClasses+4 || _classpoints->cols == 5);
        classpoints.~Map();
        new(&classpoints) Eigen::Map<const RowArray_t, Eigen::Aligned16>((const double*)_classpoints->data, _classpoints->rows, _classpoints->cols);
    }
    ref->advance(classpoints, z_height);
}

// Shift the mesh
void _sel_map_TriangularMesh_shiftMeshElements(void* self, int x_elem_shift, int y_elem_shift)
{
    auto ref = getRef(self);
    ref->shiftMeshElements(x_elem_shift, y_elem_shift);
}

// Get mesh parameters
void _sel_map_TriangularMesh_getMesh(void* self, _sel_map_matrix* vertexVector, _sel_map_matrix* vertexNormals, _sel_map_matrix* simplices)
{
    auto ref = getRef(self);
    // Store to the passed in matrices
    vertexVector->data = ref->vertexVector.data();
    vertexVector->rows = ref->vertexVector.rows();
    vertexVector->cols = ref->vertexVector.cols();
    // Get the vertexNormals
    // vertexNormals->data = ref->vertexNormals.data();
    // vertexNormals->rows = ref->vertexNormals.rows();
    // vertexNormals->cols = ref->vertexNormals.cols();
    // Skip vertexNormals for now
    simplices->data = ref->simplices.data();
    simplices->rows = ref->simplices.rows();
    simplices->cols = ref->simplices.cols();
}

// Get mesh classifications
void _sel_map_TriangularMesh_getClasses(void* self, _sel_map_confidence_list* classifications)
{
    auto ref = getRef(self);
    // Store to the passed in list
    classifications->start = (void**)ref->dirichletParamsForTerrainClass.data();
    classifications->size = ref->dirichletParamsForTerrainClass.size();
    classifications->el_size = ref->terrainClasses;
}

// Get the trimmed mesh (it should be preallocated)
void _sel_map_TriangularMesh_getTrimmed(void* self, _sel_map_matrix* vertexVector, _sel_map_matrix* simplices, _sel_map_confidence_list* classifications)
{
    using sel_map::mesh::PointWithCovArray_t;
    using sel_map::mesh::IndexArray_t;

    auto ref = getRef(self);

    // PointWithCovArray_t temporaryVertexVector;
    // IndexArray_t temporarySimplexVector;
    // std::vector<double*> temporaryClassifications;

    // // Retrieve the data
    // std::tie(temporaryVertexVector, temporarySimplexVector, temporaryClassifications) = ref->getTrimmedMesh();

    // // Map and store to buffers
    // auto storeToVertex = Eigen::Map<PointWithCovArray_t, Eigen::Aligned16>((PointWithCovArray_t::Scalar*)vertexVector->data, temporaryVertexVector.rows(), 4);
    // auto storeToSimplices = Eigen::Map<IndexArray_t, Eigen::Aligned16>((IndexArray_t::Scalar*)simplices->data, temporarySimplexVector.rows(), 3);
    // storeToVertex = temporaryVertexVector;
    // storeToSimplices = temporarySimplexVector;

    // // Iterate through and store the confidence list
    // #pragma omp parallel for schedule(static, 8)
    // for (unsigned int i = 0; i < temporaryClassifications.size(); ++i){
    //     std::copy(temporaryClassifications[i], temporaryClassifications[i]+ref->terrainClasses, (double*)classifications->start[i]);
    // }

    // Rewrite here to reduce extra copy
    // Return blanks if we have nothing
    if (!ref->activeElements.size()){
        vertexVector->rows = 0;
        simplices->rows = 0;
        classifications->size = 0;
        return;
    }
    // A random access lookup table for redirected indexes
    std::vector<int> lookupTable(ref->num_elements, -1);

    // The vertex mask, based on unique access
    std::vector<unsigned int> vertMask(ref->activeElements.size()*3);
    // The redirected simplices / face idxs. get a direct access pointer for iteration
    auto simplexBuffer = Eigen::Map<IndexArray_t, Eigen::Aligned16>((IndexArray_t::Scalar*)simplices->data, ref->activeElements.size(), 3);
    simplexBuffer = ref->simplices(ref->activeElements, Eigen::all);
    IndexArray_t::Scalar* flatIdx = simplexBuffer.data();

    // Accumulate the current key index value along with the final size of the vertMask
    unsigned int accumulator = 0;
    for (unsigned int pos = 0; pos < ref->activeElements.size()*3; ++pos){
        // If this is the first time we see this new index
        if (lookupTable[flatIdx[pos]] == -1){
            // Give it the next unique value
            lookupTable[flatIdx[pos]] = accumulator;
            // And then store the correct position to the vertex mask and dirichlet params
            vertMask[accumulator] = flatIdx[pos];
            // Then increment the accumulator
            ++accumulator;
        }
        // Regardless, update the flatIdx with the "new" index
        flatIdx[pos] = lookupTable[flatIdx[pos]];
    }

    // Now trim down the vertMask and apply
    vertMask.resize(accumulator);
    auto vertexBuffer = Eigen::Map<PointWithCovArray_t, Eigen::Aligned16>((PointWithCovArray_t::Scalar*)vertexVector->data, vertMask.size(), 4);
    vertexBuffer = ref->vertexVector(vertMask, Eigen::all);

    // Finally, the trimmed classifications
    #pragma omp parallel for schedule(static, 8)
    for (unsigned int pos = 0; pos < ref->activeElements.size(); ++pos){
        double* elemData = ref->elementsVec[ref->activeElements[pos]]->dirichletParamsForTerrainClass.data();
        std::copy(elemData, elemData+ref->terrainClasses, (double*)classifications->start[pos]);
    }

    // Set the sizes
    vertexVector->rows = vertMask.size();
    simplices->rows = ref->activeElements.size();
    classifications->size = ref->activeElements.size();
}

// Clean out the mesh points
void _sel_map_TriangularMesh_clean(void* self, uint8_t lazy)
{
    auto ref = getRef(self);
    ref->clean(lazy);
}

