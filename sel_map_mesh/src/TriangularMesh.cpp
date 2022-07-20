
#include "TriangularMesh.hpp"
#include "TriangleElement.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

#if defined(_OPENMP)
    #include <omp.h>
#endif

using sel_map::mesh::TriangularMesh;
using sel_map::mesh::RowArray_t;
using sel_map::mesh::TriangleElement;

TriangularMesh::TriangularMesh(const double bounds[3], const double originHeight, double elementLength, unsigned int pointLimit,
                              float heightPartition, double heightSafetyCheck, unsigned int terrainClasses, bool groundTruth,
                              bool verbose, unsigned int seed)
                              : Mesh<TriangleElement>(originHeight, bounds, elementLength, pointLimit, heightPartition, heightSafetyCheck, seed),
                              groundTruth(groundTruth), terrainClasses(terrainClasses), verbose(verbose)
{
    // Constructor
    int width = int(verticesLength[0]);
    auto E = 3*(width*width) - 4*width + 1;     //Edges in a square, triangular mesh
    auto V = width*width;                       //Vertices in a square, triangular mesh
    auto F = E - V + 1;                         //Euler's formula with germ g=1/2
    num_elements = F;
}

TriangularMesh::~TriangularMesh()
{
    // Destructor
    destroy();
}

void TriangularMesh::destroy()
{
    for (auto el_ptr : elementsVec){
        delete el_ptr;
    }
    elementsVec.clear();
}

void TriangularMesh::create()
{
    makeVertices();
    generateMesh();
    setUpMesh();
}

void TriangularMesh::generateMesh()
{
    /* Generates the mesh simplices for the triangular mesh. The number of total
		simplices in the mesh is found by solving for F (faces) using Euler's
		formula. */
    int width = int(verticesLength[0]);
    auto E = 3*(width*width) - 4*width + 1;     //Edges in a square, triangular mesh
    auto V = width*width;                       //Vertices in a square, triangular mesh
    auto F = E - V + 1;                         //Euler's formula with germ g=1/2

    // Prepare the simplices
    simplices.resize(F, 3);

    // Calculate and store the indices.
    #pragma omp parallel for schedule(static, 8)
    for (int el_idx = 0; el_idx < F; ++el_idx){
        simplices.row(el_idx) = genSimplices(el_idx);
    }
    
    // vertexNormals.resize(verticesLength[0]*verticesLength[1], 3);
    // vertexNormals.rowwise() = (Eigen::Array<double, 1, 3>() << 0, 0, 1).finished();
}

inline Eigen::Array<unsigned int, 3, 1> TriangularMesh::genSimplices(unsigned int triangle)
{
    int width = verticesLength[0];
    int row = int(triangle / (2 * width - 2));
    int col = int((triangle % (2 * width - 2)) / 2);
    auto LR = triangle % 2;
    return (Eigen::Array<unsigned int, 3, 1>()
            << unravelIndex(row+LR, col+LR),
               unravelIndex(row+1-LR, col+LR),
               unravelIndex(row+LR, col+1-LR)).finished();
}

void TriangularMesh::setUpMesh()
{
    /*Given a set of points and simplices making up the mesh, creates
		a single Element class for each triangular simplex of the mesh.*/
    
    // ensure the current list is cleared
    destroy();

    // Make OpenMP reduction for lists, list won't be guaranteed in order
    // #pragma omp declare reduction \
    //         (merge : \
    //          std::list<sel_map::mesh::TriangleElement> : \
    //          omp_out.splice(omp_out.end(), omp_in, omp_in.begin(), omp_in.end()))
    
    // Generate all
    elementsVec.resize(num_elements);
    dirichletParamsForTerrainClass.resize(num_elements);
    // #pragma omp parallel for reduction(merge: elements)
    #pragma omp parallel for schedule(static, 8)
    for (int el_idx = 0; el_idx < num_elements; ++el_idx){
        // Create and push back a new TriangleElement
        elementsVec[el_idx] = new TriangleElement(std::ref(*this), el_idx);
        dirichletParamsForTerrainClass[el_idx] = elementsVec[el_idx]->dirichletParamsForTerrainClass.data();
    }
}

void TriangularMesh::advance(const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points, double z_height)
{
    originHeight = z_height;
    // Add points to the mesh and get the updated elements
    std::vector<unsigned int> updateElem = addPointsToMesh(points);

    // If there's nothing left to do from here, then just skip.
    if (updateElem.size() == 0) return;

    // Generate the list of vertices that would need to be updated
    std::vector<unsigned int> updateVert(updateElem.size()*3);
    Eigen::Map<IndexArray_t>(updateVert.data(), updateElem.size(), 3) = simplices(updateElem, Eigen::all);
    auto vec_end = std::unique(updateVert.begin(), updateVert.end());
    updateVert.resize(std::distance(updateVert.begin(), vec_end));

    // Perform mesh fitting
    kernalizedMeshFitting(updateVert);

    // skip normals
    // updateMeshNormals(true);
    
    // Merge updated elements into active elements
    if(activeElements.size() == 0){
        activeElements = updateElem;
    }
    else {
        std::vector<unsigned int> temp(activeElements.size() + updateElem.size());
        auto res = std::set_union(activeElements.begin(), activeElements.end(), updateElem.begin(), updateElem.end(), temp.begin());
        temp.resize(std::distance(temp.begin(), res));
        std::sort(temp.begin(), temp.end());
        std::swap(activeElements, temp);
    }
}

// Update the trimmed
std::tuple<sel_map::mesh::PointWithCovArray_t, sel_map::mesh::IndexArray_t, std::vector<double*> >
TriangularMesh::getTrimmedMesh()
{
    // Return blanks if we have nothing
    if (!activeElements.size()){
        return std::tuple<PointWithCovArray_t, IndexArray_t, std::vector<double*> >
            (PointWithCovArray_t(), IndexArray_t(), std::vector<double*>());
    }
    // A random access lookup table for redirected indexes
    std::vector<int> lookupTable(num_elements, -1);

    // The vertex mask, based on unique access
    std::vector<unsigned int> vertMask(activeElements.size()*3);
    // The redirected simplices / face idxs. get a direct access pointer for iteration
    IndexArray_t trimmedIdx = simplices(activeElements, Eigen::all);
    IndexArray_t::Scalar* flatIdx = trimmedIdx.data();

    // Accumulate the current key index value along with the final size of the vertMask
    unsigned int accumulator = 0;
    #pragma omp parallel for schedule(static,8)
    for (unsigned int pos = 0; pos < activeElements.size()*3; ++pos){
        // If this is the first time we see this new index
        if (lookupTable[flatIdx[pos]] == -1){
            unsigned int key;
            // Capture and increment the accumulator
            #pragma omp atomic capture
            key = accumulator++;
            // Give it the next unique value
            lookupTable[flatIdx[pos]] = accumulator;
            // And then store the correct position to the vertex mask and dirichlet params
            vertMask[accumulator] = flatIdx[pos];
        }
        // Regardless, update the flatIdx with the "new" index
        flatIdx[pos] = lookupTable[flatIdx[pos]];
    }

    // Now trim down the vertMask
    vertMask.resize(accumulator);

    // The trimmed classifications
    std::vector<double*> trimmedDirichletParams(activeElements.size());
    #pragma omp parallel for schedule(static, 8)
    for (unsigned int pos = 0; pos < activeElements.size(); ++pos){
        trimmedDirichletParams[pos] = elementsVec[activeElements[pos]]->dirichletParamsForTerrainClass.data();
    }

    // Now return the trimmed values
    return std::tuple<PointWithCovArray_t, IndexArray_t, std::vector<double*> >
        (vertexVector(vertMask, Eigen::all), trimmedIdx, trimmedDirichletParams);
}

Eigen::ArrayXi TriangularMesh::generatePointBinning(const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points)
{
    // Specialized for the triangular mesh
    Eigen::VectorXi binning(points.rows());
    auto x = points.array().col(0);
    auto y = points.array().col(1);
    auto z = points.array().col(2);
    // top left, or idx 0 is at -x, +y, so flip the y first.
    auto calc_x = (x + bounds[0]) / elementLength;
    auto calc_y = (-y + bounds[1]) / elementLength;
    auto delta_x = calc_x - calc_x.floor();
    auto delta_y = calc_y - calc_y.floor();
    auto stage1_bins = calc_y.cast<int>() * (verticesLength[0]-1)*2 + calc_x.cast<int>()*2 + (delta_x + delta_y).cast<int>();
    // Folding bounds into this, anything on or out of bounds on z will result in an oob index of -1.
    auto outbounds_x = x.abs() >= bounds[0];
    auto outbounds_y = y.abs() >= bounds[1];
    auto outbounds_z = (z-originHeight).abs() >= bounds[2];
    binning = (outbounds_x || outbounds_y || outbounds_z).select(Eigen::VectorXi::Constant(points.rows(), -1), stage1_bins);
    return binning;
}

std::vector<unsigned int> TriangularMesh::addPointsToMesh(const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points)
{
    Eigen::ArrayXi bins = generatePointBinning(points);
    bool classpoints = points.cols() > 4;
    bool one_hot = points.cols() == 5;

    // Preprocess a vector list of indices from the bins
    std::vector<std::vector<unsigned int> > indices(elementsVec.size(), std::vector<unsigned int>());
    // Also generate a sparse list of elements to update
    std::vector<unsigned int> elemToUpdate;
    // This could be sped up with thrust, but not tested yet.
    for(unsigned int point_idx = 0; point_idx < points.rows(); ++point_idx){
        if (bins(point_idx) == -1) continue;
        // Push to the sparse list
        if (indices[bins(point_idx)].size() == 0) elemToUpdate.push_back(bins(point_idx));
        // Push to the index list
        indices[bins(point_idx)].push_back(point_idx);
    }

    std::sort(elemToUpdate.begin(), elemToUpdate.end());

    unsigned int accumulated_size = 0;

    // Fix a nefarious bug
    #if defined(_OPENMP)
    verifyAndUpdateGenerators();
    #endif
    // Update across all threads
    #pragma omp parallel reduction(+:accumulated_size)
    {
        int threadId = 1;
        #if defined(_OPENMP)
            threadId = omp_get_thread_num();
        #endif
        
        #pragma omp for schedule(dynamic, 4)
        for(unsigned int sparse_el_idx = 0; sparse_el_idx < elemToUpdate.size(); ++sparse_el_idx){
            auto el_idx = elemToUpdate[sparse_el_idx];
            // if (indices[el_idx].size() == 0) continue;
            
            // Just for readability, but the compiler does optimize it out
            auto& element = *(elementsVec[el_idx]);
            // Mark the element as active
            element.active = true;

            
            if(classpoints) element.addSegmentation(indices[el_idx], points, gens[threadId], one_hot);

            // Get new idx's
            indices[el_idx] = element.addPoints(indices[el_idx], points, gens[threadId]);

            accumulated_size += indices[el_idx].size();

            // to be calculated if needed
            // element.cartesianToBarycentric();
        }
    }

    this->points.conservativeResize(accumulated_size, Eigen::NoChange);
    unsigned int start = 0;
    for (auto sparse_idx_vec : elemToUpdate){
        // if(indices[sparse_idx_vec].size() == 0) continue;
        unsigned int size = indices[sparse_idx_vec].size();
        this->points.middleRows(start, size) = points(indices[sparse_idx_vec], Eigen::seqN(Eigen::fix<0>(), Eigen::fix<4>()));
        start += size;
    }
    updateTree = true;
    return elemToUpdate;
}

void TriangularMesh::kernalizedMeshFitting(std::vector<unsigned int> vertToUpdate)
{
    updateKDTreeIfNeeded();
    #pragma omp parallel for schedule(dynamic, 2)
    for(unsigned int vert_idx = 0; vert_idx < vertToUpdate.size(); ++vert_idx) {
        // Get the vertex element
        auto vertex = vertexVector.row(vertToUpdate[vert_idx]);

        // Get all points within a radius of element length
        double radius = M_SQRT2 * elementLength;
        // double radius = elementLength;
        auto closest_points = Mesh<TriangleElement>::getClosestPoints(vertex.head<2>(), AllPoints, radius, false);
        // auto closest_points = Mesh<TriangleElement>::getClosestPoints(vertex.head<2>(), 1, radius, false);
        // the above returns Eigen slices

        // If no points around vertex, skip with 0 height
        // if (closest_points.rows() < 3) continue;

        // Iterate through each row
        for (unsigned int row_idx = 0; row_idx < closest_points.first.rows(); ++row_idx){
            // continue with auto, to avoid evaluating down everything for Eigen
            auto row = closest_points.first.row(row_idx);
            auto sqr_dist = closest_points.second[row_idx];
            auto mean = row(2);
            auto var = row(3) * row(3) * (1+sqr_dist/elementLength/elementLength+2*sqrt(sqr_dist)/elementLength);
            
            // Check for numerical issues, avoid divide by 0
            // Slightly rearranged for thread safety
            if (var == 0) var = 0.001 * 0.001;

            // Initialize vertex to closest measurement if not yet initialized
            if(vertex(3) == -1){
                vertex(2) = mean;
                vertex(3) = var;
            }
            // Otherwise, Kalman update in 1D
            else {
                vertex(2) = ((var * vertex(2)) + (vertex(3) * mean)) / (var + vertex(3));
                vertex(3) = (var * vertex(3)) / (vertex(3) + var);
            }
        }
    }
}

/*mesh normals, but not used so omitted*/
void TriangularMesh::updateMeshNormals(bool updateElementNormals)
{
    // // Compute all element normals if needed
    // if(updateElementNormals){
    //     // This loop can be masked
    //     #pragma omp parallel for schedule(static, 8)
    //     for (int el_idx = 0; el_idx < elementsVec.size(); ++el_idx){
    //         auto& element = *(elementsVec[el_idx]);
    //         // element.vertices = vertexVector(element.simplex, Eigen::all);
    //         // Get normals
    //         element.computeElementNormal();
    //     }
    // }

    // // Otherwise accumulate an update
    // vertexNormals.setZero();
    // // This loop can instead use ravelindex and use activelem instead.
    // #pragma omp parallel for schedule(static, 2)
    // for(unsigned int y = 0; y < verticesLength[1]; ++y){
    //     for(unsigned int x = 0; x < verticesLength[0]; ++x){
    //         // Calculate the index
    //         auto idx = unravelIndex(y, x);

    //         // Address below
    //         int shift = (verticesLength[0]-1)*2;
    //         int base = y*shift + 2*x;
    //         // Start accumulating, unroll all positions so we can parallelize the process.
    //         if(y < verticesLength[1]-1){
    //             if(x < verticesLength[0]-1){
    //                 vertexNormals.row(idx) += elementsVec[base]->normal.array().transpose();
    //             }
    //             if(x > 0){
    //                 vertexNormals.row(idx) += elementsVec[base-1]->normal.array().transpose();
    //                 vertexNormals.row(idx) += elementsVec[base-2]->normal.array().transpose();
    //             }
    //         }
    //         if(y > 0){
    //             if(x < verticesLength[0]-1){
    //                 vertexNormals.row(idx) += elementsVec[base-shift+1]->normal.array().transpose();
    //                 vertexNormals.row(idx) += elementsVec[base-shift]->normal.array().transpose();
    //             }
    //             if(x > 0){
    //                 vertexNormals.row(idx) += elementsVec[base-shift-1]->normal.array().transpose();
    //             }
    //         }
    //     }
    // }
    // vertexNormals.rowwise().normalize();
}

// shifts into the positive x and y directions by elements.
// compute the blockwise shifts, update all element indices, deleting now invalid ones, then create new ones and reindex.
void TriangularMesh::shiftMeshElements(int x_elem_shift, int y_elem_shift)
{
    auto x = std::abs(x_elem_shift);
    auto y = std::abs(y_elem_shift);

    // If we're shifting the whole mesh, just reset
    if (x > verticesLength[0] || y > verticesLength[1]){
        vertexVector.col(2) = 0;
        vertexVector.col(3) = -1;
        // vertexNormals.rowwise() = (Eigen::Array<double, 1, 3>() << 0, 0, 1).finished();
        //Handle elements
        #pragma omp parallel for
        for (int el_idx = 0; el_idx < elementsVec.size(); ++el_idx){
            auto& element = *(elementsVec[el_idx]);
            element.reset();
        }
        // reset active elements
        activeElements.resize(0);
        return;
    }

    // Cache the values
    Eigen::Array<double, Eigen::Dynamic, 2> cachedHeights = vertexVector.rightCols<2>();

    // Calculate how to slice and shift the vertex heights
    int sliceShift = x_elem_shift;
    int blockShift = -y_elem_shift * verticesLength[0];

    // Compute the overall shift
    int shift = sliceShift + blockShift;
    int res_start = std::max(0, 0 + shift);
    int res_end = std::min(vertexVector.rows(), vertexVector.rows() + shift);
    int start = std::max(0, 0 - shift);
    int end = std::min(vertexVector.rows(), vertexVector.rows() - shift);
    // Shift
    vertexVector.rightCols<2>().middleRows(res_start, res_end - res_start) = cachedHeights.middleRows(start, end - start);
    
    // Reset new vertices
    auto z_mu = vertexVector.col(2).reshaped<Eigen::RowMajor>(verticesLength[1], verticesLength[0]);
    auto z_sigma = vertexVector.col(3).reshaped<Eigen::RowMajor>(verticesLength[1], verticesLength[0]);
    unsigned int leftReset = std::max(0, x_elem_shift);
    unsigned int rightReset = std::max(0, -x_elem_shift);
    unsigned int topReset = std::max(0, -y_elem_shift);
    unsigned int bottomReset = std::max(0, y_elem_shift);
    
    z_mu.leftCols(leftReset) = 0;
    z_mu.rightCols(rightReset) = 0;
    z_mu.topRows(topReset) = 0;
    z_mu.bottomRows(bottomReset) = 0;

    z_sigma.leftCols(leftReset) = -1;
    z_sigma.rightCols(rightReset) = -1;
    z_sigma.topRows(topReset) = -1;
    z_sigma.bottomRows(bottomReset) = -1;

    // Create a new random access elements vector
    std::vector<TriangleElement*> new_vec(elementsVec.size());
    
    // Handle elements (this can still be sped up)
    #pragma omp parallel for schedule(static, 8)// reduction(shifted_active_elements: new_activeElements)
    for (int el_idx = 0; el_idx < elementsVec.size(); ++el_idx){
        auto& element = *(elementsVec[el_idx]);
        int old_idx = element.index;
        int bin = old_idx / 2;
        int isLowerRight = old_idx % 2;

        // Compute the next location of this element
        int new_row = ravelElRow(bin) - y_elem_shift;
        int new_col = ravelElCol(bin) + x_elem_shift;
        int new_idx = unravelElIndex(new_row, new_col) * 2 + isLowerRight;
        
        // Set the element's properties
        element.index = new_idx;
        dirichletParamsForTerrainClass[element.index] = element.dirichletParamsForTerrainClass.data();
        new_vec[element.index] = &element;

        // reset the element if it's been wrapped around
        if (new_row < 0 || new_col < 0 || new_row >= verticesLength[1]-1 || new_col >= verticesLength[0]-1){
            element.reset();
            // If we were doing normals, also recompute that here
            //element.computeElementNormal();
        }
        // else if (element.active) {
        // }
    }

    // Process activeElements
    #pragma omp parallel for schedule(static, 64)
    for (int idx = 0; idx < activeElements.size(); ++idx){
        int old_idx = activeElements[idx];
        int bin = old_idx / 2;
        int isLowerRight = old_idx % 2;

        // Compute the next location of this element
        int new_row = ravelElRow(bin) - y_elem_shift;
        int new_col = ravelElCol(bin) + x_elem_shift;
        int new_idx = unravelElIndex(new_row, new_col) * 2 + isLowerRight;

        // reset the element if it's been wrapped around
        if (new_row < 0 || new_col < 0 || new_row >= verticesLength[1]-1 || new_col >= verticesLength[0]-1){
            new_idx = num_elements;
        }
        activeElements[idx] = new_idx;
    }
    activeElements.erase(std::remove(activeElements.begin(), activeElements.end(), num_elements), activeElements.end());
    std::sort(activeElements.begin(), activeElements.end());

    // Store the shifted stuff
    std::swap(elementsVec, new_vec);

    // If we were doing normals
    //updateMeshNormals(false);
}
