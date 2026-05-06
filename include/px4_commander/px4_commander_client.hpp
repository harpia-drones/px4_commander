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

    // --------------------------------------------
    //   CONSTRUCTOR / DESTRUCTOR
    // --------------------------------------------

    /**
     * @brief Construct a new Px4CommanderClient
     *
     * An internal dedicated node is created to own the service clients,
     * so the caller's node is never added to a second executor.
     *
     * @param node         Shared pointer to the caller's ROS 2 node (used for logging only)
     * @param namespace_   ROS 2 namespace prefix of the commander node (default: "px4_commander")
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
     * @brief Enable continuous trajectory setpoint publishing
     *
     * Publishes velocity-based offboard control mode and trajectory 
     * setpoints every 100 ms (heartbeat).
     *
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult enable_trajectory_setpoint();


    /**
     * @brief Disable continuous trajectory setpoint publishing
     *
     * Stops publishing velocity-based offboard control mode and trajectory 
     * setpoints.
     *
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult disable_trajectory_setpoint();


private:

    // --------------------------------------------
    //   PRIVATE METHODS
    // --------------------------------------------

    /**
     * @brief Build the fully-qualified service name with optional namespace prefix
     * @param service_name  Bare service name (e.g. "arm")
     * @return Fully-qualified name (e.g. "px4_commander/arm")
     */
    std::string _full_name(const std::string& service_name) const;

    /**
     * @brief Create a SetBool service client on the internal client node
     * @param service_name  Bare service name
     * @return The service client
     */
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
    _create_client(const std::string& service_name);

    /**
     * @brief Wait for a service to become available
     * @param client        Service client to use
     * @param service_name  Bare service name (used in log messages)
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult _wait_for_service(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client, const std::string& service_name);

    /**
     * @brief Send a SetBool request and block until response or timeout
     *
     * Uses a temporary SingleThreadedExecutor on the internal client node,
     * so the caller's node executor is never touched.
     *
     * @param client        Service client to use
     * @param service_name  Bare service name (used in log messages)
     * @param data          Value of the request data field
     * @return CommandResult with success flag and descriptive message
     */
    CommandResult _call(
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client,
        const std::string& service_name,
        bool data
    );


    // --------------------------------------------
    //   PRIVATE MEMBERS
    // --------------------------------------------

    /* ---- Caller node — used for logging only ---- */
    rclcpp::Node::SharedPtr node_;

    /* ---- Internal node — owns all service clients ---- */
    rclcpp::Node::SharedPtr client_node_;

    std::string                   namespace_;
    std::chrono::duration<double> timeout_;

    /* ---- Service clients ---- */
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_arm_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_disarm_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_offboard_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_land_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_setpoint_;

    /* ---- Service names ---- */
    static constexpr const char* _SVC_ARM                         = "arm";
    static constexpr const char* _SVC_DISARM                      = "disarm";
    static constexpr const char* _SVC_ENGAGE_OFFBOARD_MODE        = "engage_offboard_mode";
    static constexpr const char* _SVC_ENGAGE_LAND_MODE            = "engage_land_mode";
    static constexpr const char* _SVC_PUBLISH_TRAJECTORY_SETPOINT = "enable_trajectory_setpoint_publishing";
};