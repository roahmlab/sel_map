#pragma once

#include <Eigen/Dense>
#include <cstdint>
//#include <std_msgs/Float64.h> TODO use in place of double

namespace sel_map::mesh {
    typedef Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowArray_t;
    typedef Eigen::Array<double, Eigen::Dynamic, 4, Eigen::RowMajor> PointWithCovArray_t;
    typedef Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor> PointArray_t;
    typedef Eigen::Array<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> IndexArray_t;
    static const int AllPoints = -1;
    static const int NoRadius = -1;
    constexpr static const double defaultOrigin[2] = {0, 0};
    static const RowArray_t NoPoints;
}