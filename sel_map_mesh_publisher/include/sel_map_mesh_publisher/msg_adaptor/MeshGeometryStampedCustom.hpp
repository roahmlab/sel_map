#pragma once

#include <cassert>

#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <ros/static_assert.h>

#include <mesh_msgs/MeshGeometryStamped.h>

#include "MeshGeometryCustom.hpp"


// Minimally define custom types to reduce copies for the message passing
namespace sel_map
{
namespace msg_adaptor
{

  // Roughly match MeshGeometryStamped as type
  struct MeshGeometryStampedCustom
  {
    typedef std_msgs::Header _header_type;
    typedef std::string _uuid_type;
    typedef MeshGeometryCustom _mesh_geometry_type;
    _header_type header;
    _uuid_type uuid;
    _mesh_geometry_type mesh_geometry;
  };

} // namespace msg_adaptor
} // namespace sel_map

// Make it work with ros
namespace ros
{
namespace message_traits
{

  template <>
  struct IsFixedSize<sel_map::msg_adaptor::MeshGeometryStampedCustom> : FalseType {};

  template <>
  struct IsFixedSize<sel_map::msg_adaptor::MeshGeometryStampedCustom const> : FalseType {};

  template <>
  struct HasHeader<sel_map::msg_adaptor::MeshGeometryStampedCustom> : TrueType {};

  template <>
  struct HasHeader<sel_map::msg_adaptor::MeshGeometryStampedCustom const> : TrueType {};

  template<>
  struct MD5Sum<sel_map::msg_adaptor::MeshGeometryStampedCustom>
  {
    static const char* value()
    {
      // Ensure that if the definition of MeshGeometry changes we have a compile error here.
      ROS_STATIC_ASSERT(MD5Sum<mesh_msgs::MeshGeometryStamped>::static_value1 == 0x2d62dc21b3d9b8f5ULL);
      ROS_STATIC_ASSERT(MD5Sum<mesh_msgs::MeshGeometryStamped>::static_value2 == 0x28e4ee7f76a77fb7ULL);
      return MD5Sum<mesh_msgs::MeshGeometryStamped>::value();
    }

    static const char* value(const sel_map::msg_adaptor::MeshGeometryStampedCustom&) { return value(); }
  };

  template<>
  struct DataType<sel_map::msg_adaptor::MeshGeometryStampedCustom>
  {
    static const char* value()
    {
      return DataType<mesh_msgs::MeshGeometryStamped>::value();
    }

    static const char* value(const sel_map::msg_adaptor::MeshGeometryStampedCustom&) { return value(); }
  };

  template<>
  struct Definition<sel_map::msg_adaptor::MeshGeometryStampedCustom>
  {
    static const char* value()
    {
      return Definition<mesh_msgs::MeshGeometryStamped>::value();
    }

    static const char* value(const sel_map::msg_adaptor::MeshGeometryStampedCustom&) { return value(); }
  };

} // namespace message_traits

namespace serialization
{

  template<>
  struct Serializer<sel_map::msg_adaptor::MeshGeometryStampedCustom>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const sel_map::msg_adaptor::MeshGeometryStampedCustom& m)
    {
      stream.next(m.header);
      stream.next(m.uuid);
      stream.next(m.mesh_geometry);
    }

    template<typename Stream>
    inline static void read(Stream& stream, sel_map::msg_adaptor::MeshGeometryStampedCustom& m)
    {
      // Not planning on considering this, so crash out
      assert(false);
    }

    inline static uint32_t serializedLength(const sel_map::msg_adaptor::MeshGeometryStampedCustom& m)
    {
      uint32_t size = 0;
      size += serializationLength(m.header);
      size += serializationLength(m.uuid);
      size += serializationLength(m.mesh_geometry);
      return size;
    }
  };

} // namespace serialization
} // namespace ros
