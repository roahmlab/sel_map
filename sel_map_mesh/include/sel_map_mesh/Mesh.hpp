#pragma once
/**
    Declaration of the Mesh class from meshGenerator.py in C++
    @file Mesh.hpp
    @author 
    @version 2021-11-06
*/

#include "Defs.hpp"
#include "Element.hpp"
#include <list>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <nanoflann/nanoflann.hpp>

// Placing all lib elements into a sel_map namespace, partly in case this is extended upon later, partly to reduce pollution.
namespace sel_map::mesh{
    /**
     * @brief Mesh class. Contains generic mesh information and is inherited by the TriangularMesh class.
     */
    template <typename TElement_t>
    class Mesh{
        // Found using the compiler
        typedef Eigen::Block<PointWithCovArray_t, Eigen::Dynamic, 2> points_to_2d_eigen_t;
        typedef nanoflann::KDTreeEigenMatrixAdaptor<points_to_2d_eigen_t, 2, nanoflann::metric_L2_Simple> kd_tree_t;
        public:
        // Since this is really only accessed for values, and not for maths, don't eigen it.
        // (x, y) origin of the mesh
        double originHeight;
        // (x, y, z) symmetric bounds of the mesh
        double bounds[3];

        // Upper limit to # of points to consider for each element
        unsigned int pointLimit;
        // Parameters to identify a height discrepency and how many elements to select
        // (unsigned int)(1.0/heightPartition) highest height elements are selected if heightSafetyCheck condition is met
        float heightPartition; double heightSafetyCheck;

        // The metric length of one element (not hypotenuse)
        double elementLength;
        
        // number of vertices along each of the principle axis
        unsigned int verticesLength[2];

        // The points that exist within the mesh, each defined as (x, y, z)
        PointWithCovArray_t points;

        // Binning of elementIdx for each point, based on the inner space of each square grid space.
        // Eigen::VectorXi pointsBin;

        //Work the VertMesh out of here
        //VertMesh vertexMesh;
        // The vector describing all the vertexes within the mesh (x, y, mu_z, sigma_z)
        // Structured as (width*height, 4), row-major. Access [x,y] with vector[y*width+x].
        PointWithCovArray_t vertexVector;
        // Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> vertexHeights;

        // List of all Element objects
        // std::list<TElement_t> elements;
        std::vector<TElement_t*> elementsVec;
        unsigned int num_elements;
        
        // Random generator for the mesh
        std::vector<std::mt19937> gens;

        // kd_tree for getting closest points
        const unsigned int max_leaf = 4;
        points_to_2d_eigen_t points_2d;
        kd_tree_t point_index;
        bool updateTree;


        Mesh(const double originHeight, const double bounds[3], double elementLength = 1, unsigned int pointLimit = 20,
             float heightPartition = 0.3, double heightSafetyCheck = 0.5, unsigned int seed = (std::random_device())());

        #if defined(_OPENMP)
        void verifyAndUpdateGenerators();
        #endif

        void makeVertices();

        inline unsigned int unravelIndex(int row, int col){
            // Compute wraparound
            col = ((col % int(verticesLength[0])) + verticesLength[0]) % verticesLength[0];
            row = ((row % int(verticesLength[1])) + verticesLength[1]) % verticesLength[1];
            // Return the correct index
            return row * verticesLength[0] + col;
        };
        inline unsigned int ravelRow(int length){
            // Return the correct row
            return length / verticesLength[0];
        };
        inline unsigned int ravelCol(int length){
            // Return the correct col
            return length % verticesLength[0];
        };
        inline unsigned int unravelElIndex(int row, int col){
            // Compute wraparound
            col = ((col % int(verticesLength[0]-1)) + verticesLength[0]-1) % (verticesLength[0]-1);
            row = ((row % int(verticesLength[1]-1)) + verticesLength[1]-1) % (verticesLength[1]-1);
            // Return the correct index
            return row * (verticesLength[0]-1) + col;
        };
        inline unsigned int ravelElRow(int length){
            // Return the correct row
            return length / (verticesLength[0]-1);
        };
        inline unsigned int ravelElCol(int length){
            // Return the correct col
            return length % (verticesLength[0]-1);
        };

        template <typename Derived>
        Eigen::Array<bool, Eigen::Dynamic, 1> isPointWithinPolygon(const Eigen::ArrayBase<Derived>& points, const Eigen::Array<double, Eigen::Dynamic, 2>& polygonVertexCoords){
            // Process and return an eigen bool array matching the points.
            Eigen::Array<bool, Eigen::Dynamic, 1> inPolygon(points.rows());
            #pragma omp parallel for
            for(unsigned int i = 0; i < points.rows(); ++i){
                // Identify using the singular point function
                Eigen::ArrayXd point = points.row(i);
                inPolygon(i) = isSinglePointWithinPolygon(point, polygonVertexCoords);
            }
            return inPolygon;
        }
        bool isSinglePointWithinPolygon(const Eigen::Ref<Eigen::Array2d>& point, const Eigen::Array<double, Eigen::Dynamic, 2>& polygonVertexCoords);

        virtual Eigen::ArrayXi generatePointBinning(const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points);

        // Can add (renders a set of idx's to invalid states, sorts, and remove. Doesn't retain sort.)
        // but, with binning, this is redundant.
        //removePointsNotWithinBoundsOfMesh(const Eigen::MatrixBase<Derived>& point)
        void updateKDTreeIfNeeded();

        // NOT USED
        // void updatePointsFromElements();

        // todo, consider utilizing internal binning to avoid searching the entire thing
        std::pair<PointWithCovArray_t, std::vector<double> > getClosestPoints(const Eigen::Ref<const Eigen::Vector2d, Eigen::Aligned16>& targetpoint, int numPoints = AllPoints, double radius = NoRadius, bool checkUpdate = true);

        // Removes points w/in each element to free up memory after vertex height calculation.
        // Also removes points from here to free up memory.
        void clean(bool lazy = false);
    };
}