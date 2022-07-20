#pragma once
/**
    Declaration of the Element class from meshGenerator.py in C++
    @file Element.hpp
    @author 
    @version 2021/11/06
*/

#include "Defs.hpp"
#include <random>
#include <vector>

// Placing all lib elements into a sel_map namespace, partly in case this is extended upon later, partly to reduce pollution.
namespace sel_map::mesh{
    /**
     * @brief Element class. Contains generic mesh element information and is inherited by the TriangularElement class.
     */
    template <typename Tmesh_t>
    class Element{
        public:
        std::reference_wrapper<Tmesh_t> parentMesh;
        // Index of this element for it's parent mesh, like an ID
        unsigned int index;

        //The classes-length dirichlet parameters computed using MLE for terrain classification of the element
        Eigen::VectorXd dirichletParamsForTerrainClass;

        // Random generator for each element in case each element is threaded
        // std::mt19937 gen;

        /**
         * @brief Constructs a new Element object
         * 
         * @param index
         *          The index of the element within its containing mesh
         * @param vertices
         *          The cartesian coordinates for the element's (x,y) vertices, along with each mu_z, and sigma_z. (x, y, mu_z, sigma_z)
         * @param elementLength
         *          The metric distance of a one element (within its grid, not hypotenuse)
         * @param pointLimit
         *          Upper limit to number of discrete interior points & classifications to consider. Trims to this number because of computational constraints.
         * @param heightPartition
         *          If the there is a height discrepency as defined by the heightSafetyCheck threshold, only to highest 1/heightPartition interior points are considered.
         * @param heightSafetyCheck 
         *          The threshold to determine if there is a a height discrepency between the highest and lowest stored points.
         * @param classes 
         *          The number of classification classes to consider.
         * @param seed
         *          The seed for the element-specific random generator. Possible bug, due to automatic copy constructor, any copy of this element will generate the same random numbers!
         */
        // Element(const std::reference_wrapper<Tmesh_t>& parentMesh, unsigned int index, const PointArray_t& vertices, double elementLength, unsigned int pointLimit = 20,
        //         float heightPartition = 3, double heightSafetyCheck = 0.5, unsigned int classes = 23, unsigned int seed = (std::random_device())());
        // Element(const std::reference_wrapper<Tmesh_t>& parentMesh, unsigned int index, double elementLength, unsigned int pointLimit = 20,
        //         float heightPartition = 3, double heightSafetyCheck = 0.5, unsigned int classes = 23);
        Element(const std::reference_wrapper<Tmesh_t>& parentMesh, unsigned int index);
        
        // Idea, with the same method for vertarray, template classification based on classes, such that classes is actually templated in and considered at compile time
        // tradeoff between dynamic and static. Check with parker.
        ~Element();

        /**
         * @brief Adds points which are within the element as interior points
         * 
         * @param points
         *          (N, 3) - A size N set of (x, y, z) interior points to add the element.
         */
        // void addPoints(const sel_map::core::ElemArray<double, 4>& points);
        std::vector<unsigned int> addPoints(const std::vector<unsigned int>& new_points, const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points, std::mt19937& gen);

        /**
         * @brief Adds semantic segmentation probabilities associated with points which are within the element as classification tags.
         * 
         * @param points 
         *          (N, K) - A size N set of K (aka classes) semantic labeles to add to the element.
         */
        // void addSegmentation(const sel_map::core::ElemArray<double, sel_map::core::ElemArrayDynamic>& classes);
        void addSegmentation(const std::vector<unsigned int>& new_classifications, const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points, std::mt19937& gen, bool one_hot);

        /**
         * @brief Does cleanup on the object to remove points / free up space
         */
        void clean(bool lazy = false);


        // some additionally helper functions that really shouldn't be needed by anything else (i think)
        protected:

        //This can be merged into addPoints
        /**
         * @brief does height filtering if there is a sufficient difference inheight between the lowest and hieghest points in the element
         * 
         */
        // void preprocessing();
        std::vector<unsigned int> preprocessPoints(const std::vector<unsigned int>& new_points, const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points);

        //This can be merged into addPoints, addSegmentation
        /**
         * @brief Performs checks and limits the number of points per element to pointLimit
         * 
         */
        // void runSafetyCheck(sel_map::core::ElemArrayBase& obj);
        std::vector<unsigned int> trimmedPoints(const std::vector<unsigned int>& new_classifications, std::mt19937& gen);
    };
};
