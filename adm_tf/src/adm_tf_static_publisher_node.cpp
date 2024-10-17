#include <memory>
#include <string>
#include "adm_tf/adm_tf_static_publisher_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "rclcpp/rclcpp.hpp"

AdmTFStaticPublisherNode::AdmTFStaticPublisherNode(std::string name) : Node(name)
{
    // QoS settings for the broadcaster
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
        .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
        .keep_last(10)
        .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        .avoid_ros_namespace_conventions(false);

    // Create the broadcaster
    _broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this, qos_profile);

    // Create a timer to periodically send the transform (every second)
    _timer = this->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&AdmTFStaticPublisherNode::sendTransform, this));

    RCLCPP_INFO(this->get_logger(), "Static Publisher created and broadcasting transforms periodically!!");
}

void AdmTFStaticPublisherNode::sendTransform()
{
    // Define the static transform message
    geometry_msgs::msg::TransformStamped msg;
    msg.transform.translation.x = 1.0;
    msg.transform.translation.y = 0.0;
    msg.transform.translation.z = 0.0;
    
    // Set rotation (identity quaternion, no rotation)
    msg.transform.rotation.x = 0.0;
    msg.transform.rotation.y = 0.0;
    msg.transform.rotation.z = 0.0;
    msg.transform.rotation.w = 1.0;

    // Set the frame IDs and timestamp
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.child_frame_id = "camera";

  RCLCPP_INFO(this->get_logger(), "Transform sent: [%s] -> [%s], timestamp: %f", 
              msg.header.frame_id.c_str(), 
              msg.child_frame_id.c_str(), 
              rclcpp::Time(msg.header.stamp).seconds());

    // Broadcast the static transform
    _broadcaster->sendTransform(msg);
}

