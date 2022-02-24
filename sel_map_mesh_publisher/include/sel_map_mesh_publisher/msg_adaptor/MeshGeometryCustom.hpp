#pragma once

#include <cassert>

#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <ros/static_assert.h>

#include <mesh_msgs/MeshGeometry.h>

#include "VerticesArrayAdaptor.hpp"
#include "VertexNormalsArrayAdaptor.hpp"
#include "FacesArrayAdaptor.hpp"


// Minimally define custom types to reduce copies for the message passing
namespace sel_map
{
namespace msg_adaptor
{

  // Roughly match MeshGeometry as type
  struct MeshGeometryCustom
  {
    typedef VerticesArray _vertices_type;
    typedef VertexNormalsArray _vertex_normals_type;
    typedef FacesArray _faces_type;
    _vertices_type vertices;
    _vertex_normals_type vertex_normals;
    _faces_type faces;
  };

} // namespace msg_adaptor
} // namespace sel_map

// Make it work with ros
namespace ros
{
namespace message_traits
{

  template <>
  struct IsFixedSize<sel_map::msg_adaptor::MeshGeometryCustom> : FalseType {};

  template <>
  struct IsFixedSize<sel_map::msg_adaptor::MeshGeometryCustom const> : FalseType {};

  template <>
  struct HasHeader<sel_map::msg_adaptor::MeshGeometryCustom> : FalseType {};

  template <>
  struct HasHeader<sel_map::msg_adaptor::MeshGeometryCustom const> : FalseType {};


  template<>
  struct MD5Sum<sel_map::msg_adaptor::MeshGeometryCustom>
  {
    static const char* value()
    {
      // Ensure that if the definition of MeshGeometry changes we have a compile error here.
      ROS_STATIC_ASSERT(MD5Sum<mesh_msgs::MeshGeometry>::static_value1 == 0x9a7ed3efa2a35ef8ULL);
      ROS_STATIC_ASSERT(MD5Sum<mesh_msgs::MeshGeometry>::static_value2 == 0x1abaf7dcc675ed20ULL);
      return MD5Sum<mesh_msgs::MeshGeometry>::value();
    }

    static const char* value(const sel_map::msg_adaptor::MeshGeometryCustom&) { return value(); }
  };

  template<>
  struct DataType<sel_map::msg_adaptor::MeshGeometryCustom>
  {
    static const char* value()
    {
      return DataType<mesh_msgs::MeshGeometry>::value();
    }

    static const char* value(const sel_map::msg_adaptor::MeshGeometryCustom&) { return value(); }
  };

  template<>
  struct Definition<sel_map::msg_adaptor::MeshGeometryCustom>
  {
    static const char* value()
    {
      return Definition<mesh_msgs::MeshGeometry>::value();
    }

    static const char* value(const sel_map::msg_adaptor::MeshGeometryCustom&) { return value(); }
  };

} // namespace message_traits

namespace serialization
{

  template<>
  struct Serializer<sel_map::msg_adaptor::MeshGeometryCustom>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const sel_map::msg_adaptor::MeshGeometryCustom& m)
    {
      stream.next(m.vertices);
      stream.next(m.vertex_normals);
      stream.next(m.faces);
    }

    template<typename Stream>
    inline static void read(Stream& stream, sel_map::msg_adaptor::MeshGeometryCustom& m)
    {
      // Not planning on considering this, so crash out
      assert(false);
    }

    inline static uint32_t serializedLength(const sel_map::msg_adaptor::MeshGeometryCustom& m)
    {
      uint32_t size = 0;
      size += serializationLength(m.vertices);
      size += serializationLength(m.vertex_normals);
      size += serializationLength(m.faces);
      return size;
    }
  };

} // namespace serialization
} // namespace ros
