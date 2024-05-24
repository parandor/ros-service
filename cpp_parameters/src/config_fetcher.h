// ConfigFetcher.h
#ifndef PARAMETER_UPDATER_H
#define PARAMETER_UPDATER_H

#include <iostream>
#include <functional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cpprest/http_client.h>
#include <cpprest/json.h>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

using namespace web;
using namespace web::http;
using namespace web::http::client;

class ConfigFetcher : public rclcpp::Node
{
public:
    using UpdateCallback = std::function<void(const std::string &, const std::string &)>;

    ConfigFetcher(const rclcpp::NodeOptions &options) : Node("parameter_updater", options)
    {
        // Create the ROS subscriber
        subscription_no_payload_ = this->create_subscription<std_msgs::msg::String>(
            "parameter_update_no_payload", 10,
            std::bind(&ConfigFetcher::parameterUpdateNoPayloadCallback, this, std::placeholders::_1));

        // Create a subscriber to listen for configuration updates
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "parameter_update", 10,
            std::bind(&ConfigFetcher::parameterUpdateCallback, this, std::placeholders::_1));

        http_client_config client_config;
        client_ = std::make_unique<http_client>("http://localhost:5000", client_config);
    }

    void registerCallback(UpdateCallback callback)
    {
        callbacks_.push_back(callback);
    }


private:
    std::unique_ptr<http_client> client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_no_payload_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::vector<UpdateCallback> callbacks_;

    void parameterUpdateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received parameter update event: %s", msg->data.c_str());

        // Parse the incoming data
        parseData(msg->data);
    }
    void parseData(const std::string &data)
    {
        // Find the start position of the parameter object
        size_t start_pos = data.find("{");

        while (start_pos != std::string::npos)
        {
            // Find the key
            size_t key_start = data.find("\"", start_pos);   // Find the first double quote
            size_t key_end = data.find("\"", key_start + 1); // Find the next double quote

            if (key_start != std::string::npos && key_end != std::string::npos)
            {
                std::string key = data.substr(key_start + 1, key_end - key_start - 1); // Extract the key

                // Find the colon
                size_t colon_pos = data.find(":", key_end);

                if (colon_pos != std::string::npos)
                {
                    // Find the value
                    size_t value_start = data.find("\"", colon_pos + 1); // Find the double quote after the colon
                    size_t value_end = data.find("\"", value_start + 1); // Find the next double quote

                    if (value_start != std::string::npos && value_end != std::string::npos)
                    {
                        std::string value = data.substr(value_start + 1, value_end - value_start - 1); // Extract the value

                        // Invoke registered callbacks
                        for (const auto &callback : callbacks_)
                        {
                            callback(key, value);
                        }
                    }
                }
            }

            // Find the next parameter object
            start_pos = data.find("{", start_pos + 1);
        }
    }

    std::vector<std::string> split(const std::string &str, char delimiter)
    {
        std::vector<std::string> tokens;
        std::istringstream iss(str);
        std::string token;
        while (std::getline(iss, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    }

    void parameterUpdateNoPayloadCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received parameter update event");

        // Fetch parameters from the web server
        fetchParameters();
    }

    void fetchParameters()
    {
        // Make an HTTP GET request to the Flask route "/hello"
        // Ideally, this would be developed using a web 3.0 framework with optimized data packet compression
        // in order to reduce bandwidth or data for configuration consumption. Having said that, the performance
        // has to do with configuration update frequency, which should be minimal.
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

                                    // TODO: Pipe it to a callback or provide it to the target ROS node to restart and reset params.
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

RCLCPP_COMPONENTS_REGISTER_NODE(ConfigFetcher)

#endif