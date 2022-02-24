#pragma once

#include <ros/ros.h>
#include <string>

#include "msg_adaptor/MeshGeometryStampedCustom.hpp"

// Placing all lib elements into a sel_map namespace, partly in case this is extended upon later, partly to reduce pollution.
namespace sel_map::publisher{

    template <typename MeshAdaptorType>
    class TriangularMeshPublisher{
        ros::NodeHandle node_handle;
        ros::Publisher mesh_publisher;
        sel_map::msg_adaptor::MeshGeometryStampedCustom cached_message;

        public:
        MeshAdaptorType mesh_adaptor;
        
        TriangularMeshPublisher(const MeshAdaptorType& mesh_adaptor, std::string uuid, std::string frame_id, std::string mesh_topic);
        bool pubAlive();
        void publishMesh();
        unsigned int getSingleClassifications(int* buffer, unsigned int length);
    };
    
}
