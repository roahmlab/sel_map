#pragma once

#include <cstdint>
#include <cassert>

#include <ros/message_traits.h>
#include <ros/serialization.h>

#include <sel_map_mesh/Defs.hpp>
#include <Eigen/Dense>

// Minimally define custom types to reduce copies for the message passing
namespace sel_map
{
namespace msg_adaptor
{

  // Wrap a reference to simplices
  struct FacesArray
  {
    typedef Eigen::Ref<const sel_map::mesh::IndexArray_t, Eigen::Aligned16>* _faces_ptr_type;
    _faces_ptr_type faces_ptr;
  };

} // namespace msg_adaptor
} // namespace sel_map


// Make it work with ros (direct wraps are not publishable!)
namespace ros
{
namespace serialization
{

  template<>
  struct Serializer<sel_map::msg_adaptor::FacesArray>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const sel_map::msg_adaptor::FacesArray& m)
    {
      // Prepend the OUTER vector length
      uint32_t num_faces = m.faces_ptr->rows();
      stream.next(num_faces);
      // Eigen Map it to store
      const uint32_t point_len = 3*(uint32_t)sizeof(uint32_t);
      auto res = Eigen::Map<Eigen::Array<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>,
                            Eigen::Unaligned>((uint32_t*)stream.advance(num_faces*point_len), num_faces, 3);
      res = *m.faces_ptr;
    }

    template<typename Stream>
    inline static void read(Stream& stream, sel_map::msg_adaptor::FacesArray& m)
    {
      // Not planning on considering this, so crash out
      assert(false);
    }

    inline static uint32_t serializedLength(const sel_map::msg_adaptor::FacesArray& m)
    {
      // Match the serialized length as found above
      return (uint32_t)sizeof(uint32_t) + m.faces_ptr->rows() * 3 * (uint32_t)sizeof(uint32_t);
    }
  };

} // namespace serialization
} // namespace ros
