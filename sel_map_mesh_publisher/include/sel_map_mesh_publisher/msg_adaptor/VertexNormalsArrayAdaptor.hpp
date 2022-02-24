#pragma once

#include <cstdint>
#include <cassert>

#include <ros/message_traits.h>
#include <ros/serialization.h>

#include <std_msgs/Float64.h>

#include <sel_map_mesh/Defs.hpp>
#include <Eigen/Dense>

// Minimally define custom types to reduce copies for the message passing
namespace sel_map
{
namespace msg_adaptor
{

  // Wrap a pointer to vertex normals
  struct VertexNormalsArray
  {
    typedef Eigen::Ref<const sel_map::mesh::PointArray_t, Eigen::Aligned16>* _vertex_normals_ptr_type;
    _vertex_normals_ptr_type vertex_normals_ptr;
  };

} // namespace msg_adaptor
} // namespace sel_map


// Make it work with ros (direct wraps are not publishable!)
namespace ros
{
namespace serialization
{

  template<>
  struct Serializer<sel_map::msg_adaptor::VertexNormalsArray>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const sel_map::msg_adaptor::VertexNormalsArray& m)
    {
      // For cleanliness
      typedef std_msgs::Float64::_data_type float64_t;

      // Prepend the OUTER vector length
      uint32_t num_vertices = m.vertex_normals_ptr->rows();
      stream.next(num_vertices);
      // Eigen Map it to store
      const uint32_t point_len = 3*(uint32_t)sizeof(float64_t);
      auto res = Eigen::Map<Eigen::Array<float64_t, Eigen::Dynamic, 3, Eigen::RowMajor>,
                            Eigen::Unaligned>((float64_t*)stream.advance(num_vertices*point_len), num_vertices, 3);
      res = *m.vertex_normals_ptr;
    }

    template<typename Stream>
    inline static void read(Stream& stream, sel_map::msg_adaptor::VertexNormalsArray& m)
    {
      // Not planning on considering this, so crash out
      assert(false);
    }

    inline static uint32_t serializedLength(const sel_map::msg_adaptor::VertexNormalsArray& m)
    {
      // For cleanliness
      typedef std_msgs::Float64::_data_type float64_t;

      // Match the serialized length as found above
      return (uint32_t)sizeof(uint32_t) + m.vertex_normals_ptr->rows() * 3 * (uint32_t)sizeof(float64_t);
    }
  };

} // namespace serialization
} // namespace ros