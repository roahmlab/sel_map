/**
 * @file Mesh.cpp
 * @author
 * @date 2021-11-16
 * 
 * @brief Reimplements the Mesh class from meshgenerator.py
 * 
 */

#include "Mesh.hpp"
#include "TriangleElement.hpp"
#include "TriangularMesh.hpp"
#include <cmath>
#include <vector>
#include <nanoflann/nanoflann.hpp>

#if defined(_OPENMP)
    #include <omp.h>
#endif

using Eigen::Matrix;
using Eigen::Array;
using Eigen::ArrayXi;
using Eigen::Vector3d;
using std::list;

//Avoid rewriting the whole thing everytime
using sel_map::mesh::Mesh;
using sel_map::mesh::PointWithCovArray_t;

template <typename TElement_t>
Mesh<TElement_t>::Mesh(const double originHeight, const double bounds[3], double elementLength,
            unsigned int pointLimit, float heightPartition, double heightSafetyCheck, unsigned int seed)
              : pointLimit(pointLimit), heightPartition(heightPartition),
                heightSafetyCheck(heightSafetyCheck), elementLength(elementLength),
                points(), vertexVector(), elementsVec(), num_elements(0),
                points_2d(points.leftCols<2>()),
                point_index(2, std::cref(points_2d), max_leaf), updateTree(true)
{
    // copy the origin and bounds
    this->originHeight = originHeight;
    std::copy(bounds, bounds+3, this->bounds);
    // Set the verticesLength
    verticesLength[0] = std::round(bounds[0]*2 / elementLength)+1;
    verticesLength[1] = std::round(bounds[1]*2 / elementLength)+1;
    // vertexVector.resize(verticesLength[0]*verticesLength[1]);

    #if defined(_OPENMP)
        // Pregenerate random generators from the seed
        std::mt19937 gen(seed);
        std::vector<unsigned int> seeds(omp_get_max_threads()*2);
        std::generate(seeds.begin(), seeds.end(), gen);
        gens.reserve(seeds.size());
        for(auto seed : seeds){
            gens.push_back(std::mt19937(seed));
        }
    #else
        gens.reserve(1);
        gens.push_back(std::mt19937(seed));
        //Otherwise, copy the 1 random generator
    #endif
}

#if defined(_OPENMP)
template <typename TElement_t>
void Mesh<TElement_t>::verifyAndUpdateGenerators()
{
    while (omp_get_max_threads() > gens.size()){
        gens.push_back(std::mt19937(gens[0]()));
    }
}
#endif

template <typename TElement_t>
void Mesh<TElement_t>::makeVertices()
{
    // Reserve the space for the mesh and update the size
    vertexVector.resize(verticesLength[0]*verticesLength[1], 4);
    
    // Iterate through every element to set its x,y,mu_z,std_z
    #pragma omp parallel for schedule(static, 4)
    for(unsigned int y = 0; y < verticesLength[1]; ++y){
        for(unsigned int x = 0; x < verticesLength[0]; ++x){
            // Calculate the index
            auto idx = unravelIndex(y, x);
            // Get the element (contains pointer to same memory space)
            //auto elem = vertexVector[idx];
            // Using direct access instead

            // Store the initializations
            vertexVector(idx,0) = (-bounds[0] + x * elementLength);
            //* y is flipped!
            vertexVector(idx,1) = -(-bounds[1] + y * elementLength);
            // mu_z is 0
            // vertexVector(idx,2) = 0;
            // // mu_sigma is -1 when unknown
            // vertexVector(idx,3) = -1;

            // // Make all the normals straight up
            // vertexNormals(idx, Eigen::all) << 0.0, 0.0, 1.0;
        }
    }
    vertexVector.col(2) = 0;
    vertexVector.col(3) = -1;
    // vertexHeights = vertexVector.rightCols<2>();
}

template <typename TElement_t>
bool Mesh<TElement_t>::isSinglePointWithinPolygon(const Eigen::Ref<Eigen::Array2d>& point,
							const Eigen::Array<double, Eigen::Dynamic, 2>& polygonVertexCoords)
{
    // Based on the improved winding number algorithm by Dan Sunday
    // https://web.archive.org/web/20130126163405/http://geomalgorithms.com/a03-_inclusion.html
    
    // First, assume the polygonVertexCoords is in order of a described path.
    // Generate a horizontal line intersecting at the point, and create two regions, one above and one below the horizontal line.
    // This can be accomplished by shifting all vertex coords by a point.

    // Count all positive and negative transitions, where a positive transition is when two consecutive points
    // go from bottom to top, and a negative transition is vice versa. Ignore anything that intersects to the left
        // Identify transitions (and abuse auto, sorry)
    auto top = (polygonVertexCoords.col(1) > point(1));

    // Test the transitions (using cross product for edge transitions)
    auto transitions = (top.cast<int>().bottomRows(polygonVertexCoords.rows() - 1) - top.cast<int>().topRows(polygonVertexCoords.rows() - 1));
    // Edgefunction based on https://www.cs.drexel.edu/~david/Classes/Papers/comp175-06-pineda.pdf
    auto lp1 = polygonVertexCoords.topRows(polygonVertexCoords.rows() - 1);
    auto lp2 = polygonVertexCoords.bottomRows(polygonVertexCoords.rows() - 1);
    auto v1 = lp2-lp1;
    auto v2 = (-lp1).rowwise() + point.transpose();

    auto edge_transitions = v1.col(0)*v2.col(1) - v1.col(1)*v2.col(0);
    // The signs of the transitions should match, so mask with that
    int cn = ((edge_transitions*transitions.cast<double>() > 0).cast<int>() * transitions).sum();

    // Repeat the above for the final transition
    auto ftransition = top.cast<int>()(0) - top.cast<int>()(Eigen::last);
    // Edgefunction based on https://www.cs.drexel.edu/~david/Classes/Papers/comp175-06-pineda.pdf
    auto flp1 = polygonVertexCoords.row(polygonVertexCoords.rows()-1);
    auto flp2 = polygonVertexCoords.row(0);
    auto fv1 = flp2-flp1;
    auto fv2 = (-flp1).rowwise() + point.transpose();
    
    auto fedge_transition = fv1(0)*fv2(1)-fv1(1)*fv2(0);
    cn += (fedge_transition*((double)ftransition) > 0) * ftransition;

    // Return the result, only 0 (false) if it's not inside
    return cn;
}

template <typename TElement_t>
Eigen::ArrayXi Mesh<TElement_t>::generatePointBinning(const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points)
{
    Eigen::VectorXi binning(points.rows());
    auto x = points.array().col(0);
    auto y = points.array().col(1);
    auto z = points.array().col(2);
    // top left, or idx 0 is at -x, +y, so flip the y first.
    auto calc_x = (x + bounds[0]) / elementLength;
    auto calc_y = (-y + bounds[1]) / elementLength;
    auto stage1_bins = calc_y.cast<int>() * (verticesLength[0]-1) + calc_x.cast<int>();
    // Folding bounds into this, anything on or out of bounds on z will result in an oob index of -1.
    auto outbounds_x = x.abs() >= bounds[0];
    auto outbounds_y = y.abs() >= bounds[1];
    auto outbounds_z = (z-originHeight).abs() >= bounds[2];
    binning = (outbounds_x || outbounds_y || outbounds_z).select(Eigen::VectorXi::Constant(points.rows(), -1), stage1_bins);
    return binning;
}

template <typename TElement_t>
void Mesh<TElement_t>::updateKDTreeIfNeeded()
{
    #pragma omp critical
    if (updateTree){
        // Destroy the old block
        points_2d.~Block();
        // Construct it with the new data
        new(&points_2d) points_to_2d_eigen_t(points.leftCols<2>());
        point_index.index->buildIndex();
        updateTree = false;
    }
}

template <typename TElement_t>
std::pair<PointWithCovArray_t, std::vector<double> > Mesh<TElement_t>::getClosestPoints(const Eigen::Ref<const Eigen::Vector2d, Eigen::Aligned16>& targetpoint, int numPoints, double radius, bool checkUpdate)
{
    if ((numPoints == sel_map::mesh::AllPoints && radius == sel_map::mesh::NoRadius) || !numPoints || !radius)
        throw std::invalid_argument("getClosestPoints needs either a # of points or a radius!");
    
    // Build the tree
    // updated as needed.
    if (checkUpdate) updateKDTreeIfNeeded();

    // Indices to slice (std::size_t required by accessortype)
    std::vector<long int> indices;
    std::vector<double> out_dist_sqr;
    // Determine what search to do
    if (radius == sel_map::mesh::NoRadius){
        // Return numPoints points
        std::vector<long int> ret_index(numPoints);
        out_dist_sqr.resize(numPoints); 

        std::size_t num_results;
        num_results = point_index.index->knnSearch(targetpoint.data(), numPoints, &ret_index[0], &out_dist_sqr[0]);

        // In case of less points in the tree than requested:
        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results); 
        // We don't need this actually, but the search still wants it
        
        // Move to indices
        indices.resize(num_results);
        std::move(ret_index.begin(), ret_index.end(), indices.begin());
    } else {
        // Return points within the radius
        std::vector<std::pair<long int, double> > matches;
        matches.reserve(10);

        nanoflann::SearchParams params;
        params.sorted = true;

        std::size_t num_results;

        // nanoflann does L2 as squared radius, so square it here
        while ((num_results = point_index.index->radiusSearch(targetpoint.data(), radius*radius, matches, params)) < 10){
            if (radius > 3 * elementLength) break;
            radius += 0.25 * elementLength;
        }
        
        // Pull out the first elem to the return vector
        indices.reserve(num_results);
        out_dist_sqr.reserve(num_results);
        std::transform(matches.begin(), matches.end(), std::back_inserter(indices), 
                        [](const std::pair<long int, double>& p) { return p.first; });
        std::transform(matches.begin(), matches.end(), std::back_inserter(out_dist_sqr), 
                        [](const std::pair<long int, double>& p) { return p.second; });
    }
    // slice the points to return
    return std::pair<PointWithCovArray_t, std::vector<double> >(points(indices, Eigen::all), out_dist_sqr);
}

template <typename TElement_t>
void Mesh<TElement_t>::clean(bool lazy)
{
    // Clear all points, then clean all elements
    if (lazy){
        points.conservativeResize(0, Eigen::NoChange);
    }
    else{
        // No such thing as a zero array, so we just make it size one
        points.resize(1, Eigen::NoChange);
    }
    for(auto el : elementsVec){
        el->clean(lazy);
    }
}


// Forward declare needed Mesh
//template class Mesh<Element>;
template class Mesh<sel_map::mesh::TriangleElement>;
