/**
 * @file TriangularMesh.hpp
 * @author
 * @date 2021-11-16
 * 
 * @brief Definition of the TriangularMesh class from meshGenerator.py in C++
 * 
 */
#pragma once

#include "Defs.hpp"
#include <random>
#include "Mesh.hpp"
#include "TriangleElement.hpp"

// Placing all lib elements into a sel_map namespace, partly in case this is extended upon later, partly to reduce pollution.
namespace sel_map::mesh{
    class TriangularMesh : public Mesh<TriangleElement> {
        public:
        
		bool groundTruth;

		unsigned int terrainClasses;

		IndexArray_t simplices;

        bool verbose;

        // Direct access for faster export
        std::vector<double*> dirichletParamsForTerrainClass;
		// PointArray_t vertexNormals;

        std::vector<unsigned int> activeElements;
        
        TriangularMesh(const double bounds[3], const double originHeight = 0, double elementLength = 1,
                        unsigned int pointLimit = 20, float heightPartition = 3, double heightSafetyCheck = 0.5,
                        unsigned int terrainClasses = 150, bool groundTruth = false, bool verbose = false,
                        unsigned int seed = (std::random_device())());
        
        ~TriangularMesh();
        void destroy();

        /*
        Run to generate the vertices, simplices, and triangular elements making up the mesh.
		Must be called before any other function.
        */
        void create();

        /*Generates the mesh simplices for the triangular mesh. The number of total
		simplices in the mesh is found by solving for F (faces) using Euler's
		formula.*/
        void generateMesh();
        Eigen::Array<unsigned int, 3, 1> genSimplices(unsigned int triangle);

        /*Given a set of points and simplicies making up the mesh, creates
		a single Element class for each triangular simplex of the mesh.*/
        void setUpMesh();

        std::tuple<PointWithCovArray_t, IndexArray_t, std::vector<double*> > getTrimmedMesh();

        /*
        Computes the elevation map and terain property estimates given a set of sensor data
		(either range sensor measurements or categorical range sensor measurments).

		Parameters
		-----------
		points : (3, n) shape array, (x,y,z) coordinates of range sensor measurements used to compute
					elevation map
		classpoints : (3+k, n) shape array, (x,y,z,C) coordinates and terrain class probabilisties used
					to compute elevation map and terrain properties*/
        //sel_map::core::ElemArray<double, 3> points;
        //sel_map::core::ElemArray<double, sel_map::core::ElemArrayDynamic> classifications;
        Eigen::ArrayXi generatePointBinning(const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points);
	    void advance(const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points, double z_height);

        /*
        Given points of pointtype, adds them to the Element they are projected into.
        */
        //Identifies the pointtype by size_el - #cols
        std::vector<unsigned int> addPointsToMesh(const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points);

        /*
		Computes the vertex height mean and variance. Kernalized epsilon-sampling is
		used to find closest points to vertex. 1D Kalman filter update is used to
		update mean and covariance from new sensor measurments.We assume a Realsense
		noise model for the camera, as in "Analysis and Noise Modeling of the Intel 
		RealSense D435 for Mobile Robots"*/
        void kernalizedMeshFitting(std::vector<unsigned int> vertToUpdate);

        /*Adds all points contained within the inidividual TriangleElements
            to the TriangleMesh's points attribute.*/
        // void updatePointsFromElements(); -> Mesh

        /*Computed vertex height and variances within the Mesh class, but we need to transfer
		this data into the individual mesh elements.*/
    	void updateMeshNormals(bool updateElementNormals = true);

        // shifts the mesh by this many elements in the positive x or y
        void shiftMeshElements(int x_elem_shift, int y_elem_shift);

    };
};