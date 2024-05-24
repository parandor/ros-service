/* CPP Parameter Node */
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "config_fetcher.h"

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
      : Node("minimal_param_node")
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";

    this->declare_parameter("my_parameter", "world");
    this->declare_parameter("description", "default");

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Create the MinimalParam node
  auto minimal_param_node = std::make_shared<MinimalParam>();

  // Create the ConfigFetcher node
  auto updater_node = std::make_shared<ConfigFetcher>(rclcpp::NodeOptions());

  // Spin ConfigFetcher node on a separate thread
  std::thread updater_thread([&]()
                              {
                              // Register a callback with ParameterUpdater
    updater_node->registerCallback(
      [minimal_param_node](const std::string &description, const std::string &my_parameter)
      {
        RCLCPP_INFO(minimal_param_node->get_logger(), "Updated parameter: %s = %s", description.c_str(), my_parameter.c_str());
        minimal_param_node->set_parameter(rclcpp::Parameter(description, my_parameter));
      });
        
      rclcpp::spin(updater_node); });

  rclcpp::spin(minimal_param_node);
  // Wait for the updater_thread to finish
  // TODO: add clean shutdown
  updater_thread.join();

  rclcpp::shutdown();
  return 0;
}