// ParameterUpdater.h
#ifndef PARAMETER_UPDATER_H
#define PARAMETER_UPDATER_H

#include <iostream>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cpprest/http_client.h>
#include <cpprest/json.h>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

using namespace web;
using namespace web::http;
using namespace web::http::client;

class ParameterUpdater : public rclcpp::Node
{
public:
    ParameterUpdater(const rclcpp::NodeOptions &options) : Node("parameter_updater", options)
    {
        // Create the ROS subscriber
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "parameter_update", 10,
            std::bind(&ParameterUpdater::parameterUpdateCallback, this, std::placeholders::_1));

        http_client_config client_config;
        client_ = std::make_unique<http_client>("http://localhost:5000", client_config);
    }

private:
    std::unique_ptr<http_client> client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    void parameterUpdateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received parameter update event");

        // Fetch parameters from the web server
        fetchParameters();
    }

    void fetchParameters()
    {
        // Make an HTTP GET request to the Flask route "/hello"
        client_->request(methods::GET, U("/hello")).then([this](http_response response)
                                                         {
            if (response.status_code() == status_codes::OK)
            {
                // Parse the JSON response
                response.extract_json().then([this](json::value response_body) {
                    try
                    {
                        // Check if the response is an array
                        if (response_body.is_array())
                        {
                            auto parameter_array = response_body.as_array();

                            // Iterate over each parameter object in the array
                            for (const auto& param : parameter_array)
                            {
                                // Check if the parameter object has the required fields
                                if (param.has_field(U("description")) && param.has_field(U("my_parameter")))
                                {
                                    web::json::value paramCopy = param;
    
                                    std::string description = paramCopy[U("description")].as_string();
                                    std::string my_parameter = paramCopy[U("my_parameter")].as_string();

                                    // Output the received parameter
                                    RCLCPP_INFO(this->get_logger(), "Received parameter: %s = %s", "description", description.c_str());
                                    RCLCPP_INFO(this->get_logger(), "Received parameter: %s = %s", "my_parameter",  my_parameter.c_str());
                                }
                                else
                                {
                                    // Log an error if any required fields are missing
                                    RCLCPP_ERROR(this->get_logger(), "Parameter object is missing required fields");
                                }
                            }
                        }
                        else
                        {
                            // Log an error if the response is not an array
                            RCLCPP_ERROR(this->get_logger(), "JSON response is not an array");
                        }
                    }
                    catch (const std::exception &e)
                    {
                        // Log an error if there is an exception while parsing JSON
                        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON response: %s", e.what());
                    }
                }).wait();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to fetch parameters: %d", response.status_code());
            } })
            .wait();
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(ParameterUpdater)

#endif