/**
 * @brief PX4 commander client for calling arming and navigation services
 * @file px4_commander_client.hpp
 *
 * @author Thiago Marques de Oliveira
 * @date April, 2026
 */

#pragma once


/* Public includes ------------------------------------------------------- */

/* ---- General ---- */ 
#include <chrono>
#include <future>
#include <string>

/* ---- ROS2 core ---- */ 
#include <rclcpp/rclcpp.hpp>

/* ---- Service interfaces ---- */ 
#include <std_srvs/srv/set_bool.hpp>


/* Structs --------------------------------------------------------------- */

/**
 * @brief Result returned by every commander client call
 */
struct CommandResult
{
    bool success;
    std::string message;
};


/* Px4CommanderClient class ---------------------------------------------- */

class Px4CommanderClient
{
public:


    /**
     * @brief Construct a new Px4CommanderClient
     *
     * @param node         Shared pointer to the caller's ROS 2 node
     * @param namespace_   ROS 2 namespace prefix of the commander node (default: "")
     * @param timeout_sec  Time (seconds) to wait for each service response (default: 5)
     */
    explicit Px4CommanderClient(
        rclcpp::Node::SharedPtr node,
        const std::string& namespace_  = "px4_commander",
        double timeout_sec = 5.0
    );

    ~Px4CommanderClient() = default;


    // --------------------------------------------
    //   PUBLIC API
    // --------------------------------------------

    /**
     * @brief Arm the vehicle
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult arm();

    /**
     * @brief Disarm the vehicle
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult disarm();

    /**
     * @brief Engage Offboard mode
     *
     * The Px4CommanderNode publishes offboard control mode and trajectory
     * setpoints for ~2 s before sending the mode-switch command, as required
     * by PX4.
     *
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult engage_offboard_mode();

    /**
     * @brief Engage automatic landing mode (AUTO_LAND)
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult engage_land_mode();

    /**
     * @brief Enable or disable continuous trajectory setpoint publishing
     *
     * When enabled, the commander node publishes velocity-based offboard
     * control mode and trajectory setpoints every 100 ms (heartbeat).
     *
     * @param enable  True to start publishing, false to stop
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult set_trajectory_setpoint_publishing(bool enable);


private:

    // --------------------------------------------
    //   PRIVATE METHODS
    // --------------------------------------------

    /**
     * @brief Build the fully-qualified service name with optional namespace prefix
     * @param service_name  Bare service name (e.g. "arm")
     * @return Fully-qualified name (e.g. "/my_ns/arm")
     */
    std::string full_name(const std::string& service_name) const;

    /**
     * @brief Create a SetBool service client for the given service name
     * @param service_name The base name of the service
     * @return The service client
     */
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
    create_client(const std::string& service_name);

    /**
     * @brief Send a SetBool request and block until response or timeout
     * @param client The service client
     * @param service_name The base name of the service
     * @param data The boolean data to send with the request
     * @return CommandResult with success status and message
     */
    CommandResult call(
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client,
        const std::string& service_name,
        bool data
    );


    // --------------------------------------------
    //   PRIVATE MEMBERS
    // --------------------------------------------

    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    std::chrono::duration<double> timeout_;

    /* ---- Service clients ---- */
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_arm_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_disarm_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_offboard_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_land_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_setpoint_;

    /* ---- Service names ---- */
    static constexpr const char* SVC_ARM                         = "arm";
    static constexpr const char* SVC_DISARM                      = "disarm";
    static constexpr const char* SVC_ENGAGE_OFFBOARD_MODE        = "engage_offboard_mode";
    static constexpr const char* SVC_ENGAGE_LAND_MODE            = "engage_land_mode";
    static constexpr const char* SVC_PUBLISH_TRAJECTORY_SETPOINT = "publish_trajectory_setpoint";
};