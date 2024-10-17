#include <memory>
#include <string>
#include "simple_tf/simple_tf_listener_node.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

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

    // Start a timer to periodically check for transforms
    auto timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SimpleTFListenerNode::lookupTransform, this));
}

void SimpleTFListenerNode::lookupTransform()
{
    geometry_msgs::msg::TransformStamped transform;
    try {
        // Lookup the transform from "base_link" to "camera"
        transform = tf_buffer_->lookupTransform("base_link", "camera", rclcpp::Time(0));
        RCLCPP_INFO(this->get_logger(), "Transform received: translation(%f, %f, %f)",
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "Rotation: (%f, %f, %f, %f)",
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
}
