/**
 * @brief PX4 commander client for calling arming and navigation services
 * @file px4_commander_client.cpp
 *
 * @author Thiago Marques de Oliveira
 * @date April, 2026
 */


/* Private includes ------------------------------------------------------ */

#include "px4_commander/px4_commander_client.hpp"


/* Namespaces ------------------------------------------------------------ */

using namespace std::chrono_literals;


/* Px4CommanderClient class ---------------------------------------------- */


// --------------------------------------------
//   CONSTRUCTOR
// --------------------------------------------


Px4CommanderClient::Px4CommanderClient(
    rclcpp::Node::SharedPtr node,
    const std::string&      namespace_,
    double                  timeout_sec
)
: node_(node),
  namespace_(namespace_),
  timeout_(std::chrono::duration<double>(timeout_sec))
{
    cli_arm_      = this->create_client(SVC_ARM);
    cli_disarm_   = this->create_client(SVC_DISARM);
    cli_offboard_ = this->create_client(SVC_ENGAGE_OFFBOARD_MODE);
    cli_land_     = this->create_client(SVC_ENGAGE_LAND_MODE);
    cli_setpoint_ = this->create_client(SVC_PUBLISH_TRAJECTORY_SETPOINT);

    RCLCPP_INFO(node_->get_logger(), "Px4CommanderClient initialized");
}


// --------------------------------------------
//   PRIVATE METHODS
// --------------------------------------------


/**
 * @brief Build the fully-qualified service name with optional namespace prefix
 * @param service_name  Bare service name (e.g. "arm")
 * @return Fully-qualified name (e.g. "/my_ns/arm")
 */
std::string Px4CommanderClient::full_name(const std::string& service_name) const
{
    if (namespace_.empty())
    {
        return service_name;
    }

    return namespace_ + "/" + service_name;
}


/**
 * @brief Create a SetBool service client for the given service name
 * @param service_name The base name of the service
 * @return The service client
 */
rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
Px4CommanderClient::create_client(const std::string& service_name)
{
    return node_->create_client<std_srvs::srv::SetBool>(full_name(service_name));
}


/**
 * @brief Send a SetBool request and block until response or timeout
 * @param client The service client
 * @param service_name The base name of the service
 * @param data The boolean data to send with the request
 * @return CommandResult with success status and message
 */
CommandResult Px4CommanderClient::call(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client,
    const std::string& service_name,
    bool data)
{
    // Wait for service to become available
    if (!client->wait_for_service(timeout_))
    {
        const auto msg = "Service '" + full_name(service_name) + "' unavailable";
        RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    // Build and send request
    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = data;

    auto future = client->async_send_request(request);

    // Block until response or timeout
    const auto status = rclcpp::spin_until_future_complete(
        node_,
        future,
        timeout_
    );

    if (status != rclcpp::FutureReturnCode::SUCCESS)
    {
        const auto msg = "No response from service '" + full_name(service_name) + "' (timeout)";
        RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }

    auto response = future.get();
    return {response->success, response->message};
}


// --------------------------------------------
//   PUBLIC API
// --------------------------------------------


/**
 * @brief Arm the vehicle
 */
CommandResult Px4CommanderClient::arm()
{
    RCLCPP_INFO(node_->get_logger(), "Requesting ARM...");
    return call(cli_arm_, SVC_ARM, true);
}


/**
 * @brief Disarm the vehicle
 */
CommandResult Px4CommanderClient::disarm()
{
    RCLCPP_INFO(node_->get_logger(), "Requesting DISARM...");
    return call(cli_disarm_, SVC_DISARM, true);
}


/**
 * @brief Engage Offboard mode
 */
CommandResult Px4CommanderClient::engage_offboard_mode()
{
    RCLCPP_INFO(node_->get_logger(), "Requesting OFFBOARD mode...");
    return call(cli_offboard_, SVC_ENGAGE_OFFBOARD_MODE, true);
}


/**
 * @brief Engage automatic landing mode (AUTO_LAND)
 */
CommandResult Px4CommanderClient::engage_land_mode()
{
    RCLCPP_INFO(node_->get_logger(), "Requesting LAND mode...");
    return call(cli_land_, SVC_ENGAGE_LAND_MODE, true);
}


/**
 * @brief Enable or disable continuous trajectory setpoint publishing
 */
CommandResult Px4CommanderClient::set_trajectory_setpoint_publishing(bool enable)
{
    const char* action = enable ? "Enabling" : "Disabling";
    RCLCPP_INFO(node_->get_logger(), "%s trajectory setpoint publishing...", action);
    return call(cli_setpoint_, SVC_PUBLISH_TRAJECTORY_SETPOINT, enable);
}