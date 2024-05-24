/* CPP Parameter Node */
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "config_fetcher.h"

using namespace std::chrono_literals;

// For the purpose of this assignment, assume that this class is the main/top class
// that will do most work and hold most of the contents.
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

  bool restartNeeded(const std::string &description, const std::string &my_parameter)
  {
    // Implement restart check logic here
    return false; // Example implementation: No restart needed
  }

  void setRestartFlag()
  {
    // Implement restart flag setting logic here
  }

private:
  // This timer callback is for verification for the reconfiguration success.
  // It should show that there was a new parameter change or modification.
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
        // TODO: Add proper check to make sure parameters have been declared.
        // ROS node will die if params have not been declared.
        if("my_parameter" == description || "description" == description) {
             // Check if a restart is necessary
          if (minimal_param_node->restartNeeded(description, my_parameter))
          {
              // Set the restart flag
              minimal_param_node->setRestartFlag();

              // Log the restart requirement
              RCLCPP_INFO(minimal_param_node->get_logger(), "Restart required due to parameter update.");

              // Perform any additional actions required before restarting
              // For example, cleanup resources, close connections, etc.

              // Shutdown the ROS node
              rclcpp::shutdown();
          } else {
            minimal_param_node->set_parameter(rclcpp::Parameter(description, my_parameter));
            RCLCPP_INFO(minimal_param_node->get_logger(), "Updated parameter: %s = %s", description.c_str(), my_parameter.c_str());
          }
        }
      });
        
      rclcpp::spin(updater_node); });

  rclcpp::spin(minimal_param_node);
  // Wait for the updater_thread to finish
  // TODO: add clean shutdown
  updater_thread.join();

  rclcpp::shutdown();
  return 0;
}