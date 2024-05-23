// MyClass.h
#ifndef PARAMETER_UPDATER_H
#define PARAMETER_UPDATER_H

#include <rclcpp/rclcpp.hpp>
#include <cpprest/http_client.h>
#include <cpprest/http_listener.h>
#include <cpprest/json.h>
#include <iostream>

using namespace web;
using namespace web::http;
using namespace web::http::client;
using namespace web::http::experimental::listener;

class ParameterUpdater : public rclcpp::Node
{
public:
    ParameterUpdater() : Node("parameter_updater")
    {
        http_client_config client_config;
        client_ = std::make_unique<http_client>("http://localhost:5000", client_config);

        // Set up the HTTP listener
        http_listener_ = std::make_unique<http_listener>("http://0.0.0.0:6000/update_parameters");
        http_listener_->support(methods::POST, std::bind(&ParameterUpdater::handle_post, this, std::placeholders::_1));
        http_listener_->open().wait();
    }

    ~ParameterUpdater()
    {
        http_listener_->close().wait();
    }

private:
    std::unique_ptr<http_client> client_;
    std::unique_ptr<http_listener> http_listener_;

    void handle_post(http_request request)
    {
        request.extract_json().then([=](json::value request_body)
                                    {
            try
            {
                // Check if the trigger_update flag is set to true
                bool trigger_update = request_body[U("trigger_update")].as_bool();
                if (trigger_update)
                {
                    // Fetch parameters from the web service
                    fetch_parameters();
                }

                // Respond with success
                request.reply(status_codes::OK, U("Parameters updated successfully"));
            }
            catch (const std::exception &e)
            {
                // Respond with error
                request.reply(status_codes::BadRequest, U("Failed to update parameters: ") + utility::conversions::to_string_t(e.what()));
            } })
            .wait();
    }

    void fetch_parameters()
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
                        // Extract parameters from the JSON response
                        auto hello = response_body[U("hello")];
                        for (auto const &param : hello.as_array())
                        {
                             web::json::value paramCopy = param;
    
                            std::string description = paramCopy[U("description")].as_string();
                            std::string my_parameter = paramCopy[U("my_parameter")].as_string();
                            // Store the parameters as needed
                            RCLCPP_INFO(this->get_logger(), "Received parameter: %s = %s", description.c_str(), my_parameter.c_str());
                        }
                    }
                    catch (const std::exception &e)
                    {
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

#endif