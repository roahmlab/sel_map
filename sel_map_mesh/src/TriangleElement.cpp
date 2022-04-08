/**
 * @file TriangleElement.cpp
 * @author Adam Li (adamli@umich.edu)
 * @date 2021-11-12
 * 
 * @brief Implements TriangleElement from meshGenerator.py. 
 * 
 */

#include "TriangleElement.hpp"
#include <cstring>
#include <cstdlib>
#include <Eigen/Dense>
#include "TriangularMesh.hpp"

// #include "MathUtils.hpp"
// namespace MathUtils = sel_map::mesh::MathUtils;

//Avoid rewriting the whole thing everytime
using sel_map::mesh::TriangularMesh;
using sel_map::mesh::TriangleElement;
using sel_map::mesh::Element;

// TriangleElement::TriangleElement(const std::reference_wrapper<TriangularMesh>& parentMesh, unsigned int index, double elementLength, unsigned int pointLimit, float heightPartition,
//                     double heightSafetyCheck, unsigned int classes)
//                       : Element<TriangularMesh>(parentMesh, index, elementLength, pointLimit, heightPartition, heightSafetyCheck, classes)
//                     //   , lambdas()
TriangleElement::TriangleElement(const std::reference_wrapper<TriangularMesh>& parentMesh, unsigned int index)
                      : Element<TriangularMesh>(parentMesh, index)
{
    // Initializer
    active = false;
}

TriangleElement::~TriangleElement()
{
    // Destructor
}

void TriangleElement::reset()
{
    active = false;
    dirichletParamsForTerrainClass.setZero();
}

/**
 * @brief Computes the Barycentric coordinates of all points within the element from Cartesian points.
 */
// void TriangleElement::cartesianToBarycentric()
// {
//     // Auto keywords in eigen are really dangerous, oops
//     // These are all effectively Map<vector2d, unaligned> types
//     auto vertices = parentMesh.get().vertexVector(simplex(), Eigen::all);
//     auto v1 = vertices(0, Eigen::seqN(Eigen::fix<0>, Eigen::fix<2>));
//     auto v2 = vertices(0, Eigen::seqN(Eigen::fix<0>, Eigen::fix<2>));
//     auto v3 = vertices(0, Eigen::seqN(Eigen::fix<0>, Eigen::fix<2>));
//     // These two variables should never be touched again! they're just intermediates
//     auto area_iexp1 = v2 - v1;
//     auto area_iexp2 = v3 - v1;
//     double area = EigenMathUtil::crossProduct2D(area_iexp1, area_iexp2);

//     // This is effectively Map<ArrayXd, aligned> type
//     auto eigenPointsArray = points.getEigenArray();
//     auto eigenLambdaArray = lambdas.reserve(points.length(), true).getEigenArray();
    
//     // The following is simplified to keep eveything in scalar math
//     // alpha = abs(np.cross(p2-self.points[:,0:2], p3-self.points[:,0:2]) / area)
//     // beta  = abs(np.cross(p1-self.points[:,0:2], p3-self.points[:,0:2]) / area)
//     // gamma = abs(np.cross(p1-self.points[:,0:2], p2-self.points[:,0:2]) / area)
//     // self.lambdas = np.array([alpha, beta, gamma])
    
//     double v1_x_v2 = MathUtils::crossProduct2D(v1, v2);
//     double v1_x_v3 = MathUtils::crossProduct2D(v1, v3);
//     double v2_x_v3 = MathUtils::crossProduct2D(v2, v3);

//     // update alpha
//     eigenLambdaArray.col(0) = ((v2_x_v3
//                                  + (v2(1) - v3(1)) * eigenPointsArray.col(0)
//                                  + (v3(0) - v2(0)) * eigenPointsArray.col(1)
//                               ) / area).abs();

//     // update beta
//     eigenLambdaArray.col(1) = ((v1_x_v3
//                                  + (v1(1) - v3(1)) * eigenPointsArray.col(0)
//                                  + (v3(0) - v1(0)) * eigenPointsArray.col(1)
//                               ) / area).abs();

//     // update gamma
//     eigenLambdaArray.col(2) = ((v1_x_v2
//                                  + (v1(1) - v2(1)) * eigenPointsArray.col(0)
//                                  + (v2(0) - v1(0)) * eigenPointsArray.col(1)
//                               ) / area).abs();
// }

void TriangleElement::computeElementNormal()
{
    // auto vertices = parentMesh.get().vertexVector(simplex(), Eigen::all);
    // auto a = vertices(2, Eigen::seqN(Eigen::fix<0>(), Eigen::fix<3>()));
    // auto b = vertices(1, Eigen::seqN(Eigen::fix<0>(), Eigen::fix<3>()));
    // auto c = vertices(0, Eigen::seqN(Eigen::fix<0>(), Eigen::fix<3>()));

    // auto v1 = a - b;
    // auto v2 = c - b;
    // normal = v1.matrix().cross(v2.matrix());
}


Eigen::Block<Eigen::Array<unsigned int, -1, 3, 1>, 1, 3, true> TriangleElement::simplex()
{
    return parentMesh.get().simplices.row(index);
}
