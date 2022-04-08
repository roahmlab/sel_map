#pragma once
/**
    Declaration of the TriangleElement class from meshGenerator.py in C++
    @file TriangleElement.hpp
    @author 
    @version 2021/11/06
*/

#include "Defs.hpp"
#include <random>
#include <array>
#include "Element.hpp"
// #include "TriangularMesh.hpp"

// Placing all lib elements into a sel_map namespace, partly in case this is extended upon later, partly to reduce pollution.
namespace sel_map::mesh{
    /**
     * @brief Child class of the Element class. Includes more specific attribute information regarding a triangular mesh element.
     * 
     * @tparam Tclasses 
     *          The number of classification classes to consider.
     */
    class TriangularMesh;

    class TriangleElement : public Element<TriangularMesh> {
        public:

        bool active; // refactor
        
        TriangleElement(const std::reference_wrapper<TriangularMesh>& parentMesh, unsigned int index);
        
        //Destructor just to delete the dirichlet params for terrain class.
        //alternately, we can try template tricks - going to use Eigen.
        ~TriangleElement();

        /**
         * @brief Computes the Barycentric coordinates of all points within the element from Cartesian points.
         */
        // void cartesianToBarycentric();

        void computeElementNormal();

        Eigen::Block<Eigen::Array<unsigned int, -1, 3, 1>, 1, 3, true> simplex();
        
        void reset();
    };
};
