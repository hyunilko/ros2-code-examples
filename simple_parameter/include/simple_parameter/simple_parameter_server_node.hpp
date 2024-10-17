#ifndef __SIMPLE_PARAMETER_SERVER_NODE_HPP__
#define __SIMPLE_PARAMETER_SERVER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"

class SimpleParameterServerNode : public rclcpp::Node {

public:

    SimpleParameterServerNode();

private:

    void parameters_init();

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
};

#endif // __SIMPLE_PARAMETER_SERVER_NODE_HPP__