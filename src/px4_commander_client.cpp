/**
 * @brief PX4 commander client for calling arming and navigation services
 * @file px4_commander_client.cpp
 *
 * @author Thiago Marques de Oliveira
 * @date April, 2026
 */


/* Private includes ------------------------------------------------------ */

#include "px4_commander/px4_commander_client.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>


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
    /* 
     * Create a dedicated internal node to own the service clients.
     * This prevents the caller's node from being added to a second executor
     * when spin_until_future_complete is called inside call(). 
     */
    client_node_ = std::make_shared<rclcpp::Node>(
        std::string(node->get_name()) + "_commander_client"
    );

    // Create all clients
    this->cli_arm_      = this->_create_client(_SVC_ARM);
    this->cli_disarm_   = this->_create_client(_SVC_DISARM);
    this->cli_offboard_ = this->_create_client(_SVC_ENGAGE_OFFBOARD_MODE);
    this->cli_land_     = this->_create_client(_SVC_ENGAGE_LAND_MODE);
    this->cli_setpoint_ = this->_create_client(_SVC_PUBLISH_TRAJECTORY_SETPOINT);

    RCLCPP_INFO(node_->get_logger(), "Px4CommanderClient initialized");
}


// --------------------------------------------
//   PRIVATE METHODS
// --------------------------------------------


/**
 * @brief Build the fully-qualified service name with optional namespace prefix
 */
std::string Px4CommanderClient::_full_name(const std::string& service_name) const
{
    if (namespace_.empty())
    {
        return service_name;
    }

    return namespace_ + "/" + service_name;
}


/**
 * @brief Create a SetBool service client on the internal client node
 */
rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
Px4CommanderClient::_create_client(const std::string& service_name)
{
    return client_node_->create_client<std_srvs::srv::SetBool>(this->_full_name(service_name));
}


/**
 * @brief Wait for a service to become available
 */
CommandResult Px4CommanderClient::_wait_for_service(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client, const std::string& service_name)
{
    if (!client->wait_for_service(timeout_))
    {
        const auto msg = "Service '" + this->_full_name(service_name) + "' unavailable";
        RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
        return {false, msg};
    }
    else
    {
        return {true, "Service available"};
    }
}


/**
 * @brief Send a SetBool request and block until response or timeout
 */
CommandResult Px4CommanderClient::_call(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client,
    const std::string& service_name,
    bool data)
{
    // Wait for service to become available
    CommandResult wait_result = this->_wait_for_service(client, service_name);
    if (!wait_result.success)
        return wait_result;

    // Build and send request
    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = data;

    auto future = client->async_send_request(request);

    // Spin only the internal client node — the caller's node is never touched
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(client_node_);

    const auto status = executor.spin_until_future_complete(future, timeout_);

    executor.remove_node(client_node_);

    if (status != rclcpp::FutureReturnCode::SUCCESS)
    {
        const auto msg = "No response from service '" + this->_full_name(service_name) + "' (timeout)";
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
    return this->_call(this->cli_arm_, _SVC_ARM, true);
}


/**
 * @brief Disarm the vehicle
 */
CommandResult Px4CommanderClient::disarm()
{
    RCLCPP_INFO(node_->get_logger(), "Requesting DISARM...");
    return this->_call(this->cli_disarm_, _SVC_DISARM, true);
}


/**
 * @brief Engage Offboard mode
 */
CommandResult Px4CommanderClient::engage_offboard_mode()
{
    RCLCPP_INFO(node_->get_logger(), "Requesting OFFBOARD mode...");
    return this->_call(this->cli_offboard_, _SVC_ENGAGE_OFFBOARD_MODE, true);
}


/**
 * @brief Engage automatic landing mode (AUTO_LAND)
 */
CommandResult Px4CommanderClient::engage_land_mode()
{
    RCLCPP_INFO(node_->get_logger(), "Requesting LAND mode...");
    return this->_call(this->cli_land_, _SVC_ENGAGE_LAND_MODE, true);
}


/**
 * @brief Enable continuous trajectory setpoint publishing
 */
CommandResult Px4CommanderClient::enable_trajectory_setpoint()
{
    RCLCPP_INFO(node_->get_logger(), "Enabling trajectory setpoint...");
    return this->_call(this->cli_setpoint_, _SVC_PUBLISH_TRAJECTORY_SETPOINT, true);
}


/**
 * @brief Disable continuous trajectory setpoint publishing
 */
CommandResult Px4CommanderClient::disable_trajectory_setpoint()
{
    RCLCPP_INFO(node_->get_logger(), "Disabling trajectory setpoint...");
    return this->_call(this->cli_setpoint_, _SVC_PUBLISH_TRAJECTORY_SETPOINT, false);
}