#include <memory>
#include <string>
#include "simple_tf/simple_tf_listener_node.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"

SimpleTFListenerNode::SimpleTFListenerNode(std::string name) : Node(name)
{
    // Initialize the TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Listener created!!");

    // Create a timer to periodically check for transforms (every second)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SimpleTFListenerNode::lookupTransform, this));
}

void SimpleTFListenerNode::lookupTransform()
{
    geometry_msgs::msg::TransformStamped transform;
    try {
        // Lookup the transform from "base_link" to "camera"
        transform = tf_buffer_->lookupTransform("base_link", "camera", tf2::TimePointZero);
        
        // Print the transform translation
        RCLCPP_INFO(this->get_logger(), "Transform received: translation (x: %f, y: %f, z: %f)",
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z);
        
        // Print the transform rotation (as quaternion)
        RCLCPP_INFO(this->get_logger(), "Rotation (qx: %f, qy: %f, qz: %f, qw: %f)",
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
    }
    catch (const tf2::TransformException & ex) {
        // Warn if the transform is not available
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
}
