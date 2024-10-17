#ifndef __SIMPLE_TF_STATIC_PUBLISHER_NODE_HPP__
#define __SIMPLE_TF_STATIC_PUBLISHER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class AdmTFStaticPublisherNode : public rclcpp::Node {

public:
    // Constructor
    AdmTFStaticPublisherNode(std::string name = "adm_tf_static_publisher");

private:
    // Function to send the transform periodically
    void sendTransform();

    // Static transform broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _broadcaster;

    // Timer to trigger transform broadcasting
    rclcpp::TimerBase::SharedPtr _timer;
};

#endif // __SIMPLE_TF_STATIC_PUBLISHER_NODE_HPP__
