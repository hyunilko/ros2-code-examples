#include "simple_client/simple_client_node.hpp"

using GetImageSrv = simple_interfaces::srv::GetImage;
using namespace std::chrono_literals;

SimpleClientNode::SimpleClientNode(std::string name) : Node(name)
{

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    _client = this->create_client<GetImageSrv>("get_image", qos_profile);

    while (!_client->wait_for_service(100ms)){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
            assert(0);
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
    }

    RCLCPP_INFO(this->get_logger(), "Client created!!");
}


void SimpleClientNode::run_request_loop()
{

    rclcpp::WallRate loop_rate(250ms);
    while(rclcpp::ok()){

        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        std::shared_ptr<GetImageSrv::Request> request = std::make_shared<GetImageSrv::Request>();

        request->res_w = 480;
        request->res_h = 320;

        _client->async_send_request(
            request,
            [this, t1](rclcpp::Client<GetImageSrv>::SharedFuture future_response) {
                std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
                auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

                auto response = future_response.get();
                if (response) {
                    RCLCPP_INFO(this->get_logger(), "Got response in %ld microseconds", request_duration);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed :(");
                }
            });

        loop_rate.sleep();
    }

}

