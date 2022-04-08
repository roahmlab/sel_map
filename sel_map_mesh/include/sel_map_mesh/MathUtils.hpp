#pragma once
/**
 * @file MathUtils.hpp
 * @author Adam Li (adamli@umich.edu)
 * @date 2021-11-12
 * 
 * @brief Includes a variety of utility functions for Eigen objects of the same type.
 * 
 */

#include <Eigen/Core>

namespace sel_map::mesh::MathUtils {
    
    template <typename Derived1, typename Derived2>
    inline double crossProduct2D(const Eigen::EigenBase<Derived1>& x, const Eigen::EigenBase<Derived2>& y){
        return x.derived()(0)*y.derived()(1) - x.derived()(1)*y.derived()(0);
    }

    // Convert boolean masks to a sequence usable for slicing vectors
    template <typename Derived>
    inline std::vector<unsigned int>& seqLogical(const Eigen::ArrayBase<Derived>& mask, std::vector<unsigned int>& sequence){
        if (mask.cols() != 1) throw std::invalid_argument("Mask should be single dim column array!");
        
        // Create the result
        // std::vector<unsigned int> sequence;
        sequence.clear();
        sequence.resize(mask.count());

        unsigned int accumulator = 0;
        auto itr = sequence.begin();

        for (auto val : mask){
            if(val) *(itr++) = (accumulator);
            ++accumulator;
        }

        // Make a sequence
        // Eigen::Array<unsigned int, Eigen::Dynamic, 1>::Map(&sequence[0], sequence.size())
        //      = mask.template cast<bool>().template cast<unsigned int>() * Eigen::Array<unsigned int, Eigen::Dynamic, 1>::LinSpaced(mask.rows(), 1, mask.rows());
        
        // // remove the 0's
        // auto seq_end = std::remove(sequence.begin(), sequence.end(), 0);

        // // Isolate the actual values
        // sequence.resize(seq_end - sequence.begin());
        
        // // Subtract 1 to make it valid
        // Eigen::Array<unsigned int, Eigen::Dynamic, 1>::Map(&sequence[0], sequence.size()) -= 1;

        return sequence;
    }
}