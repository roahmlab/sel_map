/**
    Definition/Implementation of the Element class from meshGenerator.py in C++
    @file Element.cpp
    @author 
    @version 2021/11/06
*/

#include "Element.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "TriangularMesh.hpp"

//Avoid rewriting the whole thing everytime
using sel_map::mesh::Element;
using sel_map::mesh::RowArray_t;

template <typename Tmesh_t>
Element<Tmesh_t>::Element(const std::reference_wrapper<Tmesh_t>& parentMesh, unsigned int index)
                  : parentMesh(parentMesh), index(index)
{
    // Class initializer
    // points.reserve(pointLimit);

    // Set the dirichlet params for geometric confidence to zero
    dirichletParamsForTerrainClass = Eigen::ArrayXd::Zero(parentMesh.get().terrainClasses);
};

template <typename Tmesh_t>
Element<Tmesh_t>::~Element()
{
    // Class destructor
}

template <typename Tmesh_t>
std::vector<unsigned int> Element<Tmesh_t>::addPoints(const std::vector<unsigned int>& new_points, const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points, std::mt19937& gen)
{
    // Short circuit if there are no points
    if(new_points.size() == 0) return new_points;

    // Store a reference wrapper to points so we can change it as needed
    auto points_wrapper = std::cref(new_points);

    // Preprocess if needed
    std::vector<unsigned int> preprocessed = preprocessPoints(new_points, points);
    if (preprocessed.size() > 0){
        points_wrapper = std::cref(preprocessed);
    }

    // Trim if needed
    std::vector<unsigned int> trimmed = trimmedPoints(points_wrapper.get(), gen);
    if (trimmed.size() > 0){
        points_wrapper = std::cref(trimmed);
    }

    // Temp
    // this->points.append(points(points_wrapper.get(), Eigen::seqN(Eigen::fix<0>,Eigen::fix<4>)));
    return points_wrapper.get();
};

template <typename Tmesh_t>
void Element<Tmesh_t>::addSegmentation(const std::vector<unsigned int>& new_classifications, const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points, std::mt19937& gen, bool one_hot)
{
    // Short circuit if there are no classifications
    if(new_classifications.size() == 0) return;

    // Store a reference wrapper to classifications so we can change it if trimming is needed
    auto classes = std::cref(new_classifications);

    // Trim if we need to
    std::vector<unsigned int> trimmed = trimmedPoints(new_classifications, gen);
    if (trimmed.size() > 0){
        classes = std::cref(trimmed);
    }

    // Transform classifications into one-hot encodings, add these one-hots
    // to the dirichlet parameters to perform Dirichlet MAP update.
    if(one_hot){
        for(auto class_idx : classes.get()){
            ++dirichletParamsForTerrainClass((int)points(class_idx, 4));
        }
    } else{
        for(auto class_idx : classes.get())
        {
            Eigen::Index idx;
            points(class_idx, Eigen::seq(Eigen::fix<4>,Eigen::last)).maxCoeff(&idx);
            ++dirichletParamsForTerrainClass(idx);
        }
    }

    // If this is groundTruth, one-hot encode.
    if (parentMesh.get().groundTruth){
        Eigen::Index idx;
        dirichletParamsForTerrainClass.maxCoeff(&idx);
        dirichletParamsForTerrainClass.setZero();
        dirichletParamsForTerrainClass(idx) = 1;
    }
};

template <typename Tmesh_t>
void Element<Tmesh_t>::clean(bool lazy)
{
    // if (lazy){
    //     points.setEmpty();
    //     classifications.setEmpty();
    // }
    // else{
    //     points.clear();
    //     classifications.clear();
    // }
};

template <typename Tmesh_t>
std::vector<unsigned int> Element<Tmesh_t>::preprocessPoints(const std::vector<unsigned int>& new_points, const Eigen::Ref<const RowArray_t, Eigen::Aligned16>& points)
{
    std::vector<unsigned int> preprocessed;

    //Create lambda function for sort
    auto sortByHeight = [points](unsigned int a, unsigned int b) -> bool
                        { return points.coeff(a,2) < points.coeff(b,2); };
    
    //Find the minmax first, since this is 1.5n
    auto minmax = std::minmax_element(new_points.begin(), new_points.end(), sortByHeight);
    //Check the height delta, second is the max, first is the min.
    if(points.coeff((*minmax.second), 2) - points.coeff((*minmax.first), 2) >= parentMesh.get().heightSafetyCheck){
        preprocessed = new_points;
        //If this fails, then do an inplace sort in reverse and in place trim of the vector by indices
        std::sort(preprocessed.rbegin(), preprocessed.rend(), sortByHeight);
        
        // int num_chosen_points = std::round((float)preprocessed.size() / heightPartition);
        int num_chosen_points = std::ceil((float)preprocessed.size() / parentMesh.get().heightPartition);
        preprocessed.resize(num_chosen_points);
    }
    return preprocessed;
}

template <typename Tmesh_t>
std::vector<unsigned int> Element<Tmesh_t>::trimmedPoints(const std::vector<unsigned int>& idxs, std::mt19937& gen)
{
    std::vector<unsigned int> trimmed;

    // Trim if we need to
    if (idxs.size() > parentMesh.get().pointLimit){
        trimmed = idxs;
        std::shuffle(trimmed.begin(), trimmed.end(), gen);
        trimmed.resize(parentMesh.get().pointLimit);
    }
    return trimmed;
};

template class Element<sel_map::mesh::TriangularMesh>;