/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Modification Copyright 2026 Thiago Marques de Oliveira
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief PX4 commander node for setting navigation and arming states
 * @file px4_commander_node.hpp
 *
 * @author Thiago Marques de Oliveira
 * @date April, 2026
 */


#ifndef PX4_COMMANDER_SERVER_NODE__HPP
#define PX4_COMMANDER_SERVER_NODE__HPP


/* Includes -------------------------------------------------------------- */

/* ---- General ---- */ 
#include <optional>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <functional>

/* ---- ROS2 core ---- */ 
#include <rclcpp/rclcpp.hpp>


/* ---- Message interfaces ---- */ 

// Publishers
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

// Subscribers
#include <px4_msgs/msg/vehicle_status.hpp>


/* ---- Service interfaces ---- */ 
#include <std_srvs/srv/set_bool.hpp>


/* ---- PX4 custom mode ---- */ 
#include "px4_commander/px4_custom_mode.hpp"


/* Structs --------------------------------------------------------------- */

/**
 * @brief Arguments for vehicle commands
 */
struct VehicleCommandArgs
{
    uint16_t command; 
    std::optional<float> param1 = std::nullopt;
    std::optional<float> param2 = std::nullopt;
    std::optional<float> param3 = std::nullopt;
    std::optional<float> param4 = std::nullopt;
    std::optional<float> param5 = std::nullopt;
    std::optional<float> param6 = std::nullopt;
    std::optional<float> param7 = std::nullopt;
};


/* Px4CommanderServer class ---------------------------------------------- */

class Px4CommanderServerNode
: public rclcpp::Node
{
public:

    /**
     * @brief Construct a new Px4CommanderServerNode
     * @param node_name    Name of the commander node
     */
    explicit Px4CommanderServerNode(const std::string node_name);

    // Detructor
    ~Px4CommanderServerNode() = default;

private:

    // --------------------------------------------
    //   CLOCK
    // --------------------------------------------

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    
    // --------------------------------------------
    //   BUFFERS
    // --------------------------------------------

    /* ---- Vehicle's states  ---- */ 

    // Arming state
    std::atomic<uint8_t> current_arming_state_{
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED
    };

    // Nav state
    std::atomic<uint8_t> current_nav_state_{0}; // Manual mode as default

    // PX4 synced timestamp
    std::atomic<uint64_t> timestamp_{0};


    /* ---- Logic variables ---- */ 

    bool is_simulation_; // is_simulation ? true : false

    rclcpp::Duration pre_offboard_waiting_time_{
        rclcpp::Duration::from_seconds(2.0)  // 2 seconds
    };
    
    std::atomic<bool> publish_trajectory_setpoint_{true};
        
    std::mutex wait_success_mutex_;
    std::condition_variable wait_success_cv_;
    std::chrono::seconds cv_response_timeout_{
        std::chrono::seconds(5)  // 5 seconds
    }; 


    // --------------------------------------------
    //   CALLBACK GROUPS
    // --------------------------------------------

    rclcpp::CallbackGroup::SharedPtr control_group_;
    rclcpp::CallbackGroup::SharedPtr service_group_;


    // --------------------------------------------
    //   TIMERS
    // --------------------------------------------

    rclcpp::TimerBase::SharedPtr send_heartbeat_signal_timer_;


    // --------------------------------------------
    //   PUBLISHERS
    // --------------------------------------------

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;


    // --------------------------------------------
    //   SUBSCRIPTIONS
    // --------------------------------------------

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;


    // --------------------------------------------
    //   SERVICE SERVERS
    // --------------------------------------------
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_trajectory_setpoint_publishing_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr disarm_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr engage_offboard_mode_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr engage_land_mode_service_;


    // --------------------------------------------
    //   METHODS
    // --------------------------------------------

    /* ---- PX4 functions ---- */ 

    /**
     * @brief Publish vehicle commands
     * @param args VehicleCommand struct with command and parameters 1-7
     */
    void publish_vehicle_command(const VehicleCommandArgs& args);

    /**
     * @brief Send a command to arm the vehicle
     */
    void arm();

    /**
     * @brief Send a command to disarm the vehicle
     */
    void disarm();

    /**
     * @brief Send a command to land the vehicle
     */
    void land();
    
    /**
     * @brief Send a command to engage offboard mode
     */
    void engage_offboard_mode();

    /**
     * @brief Publish the offboard control mode to be controlled by POSITION
     */
    void publish_offboard_control_mode_by_position();

    /**
     * @brief Publish the offboard control mode to be controlled by VELOCITY
     */
    void publish_offboard_control_mode_by_velocity();

    /**
     * @brief Publish a trajectory setpoint by POSITION
     */
    void publish_trajectory_setpoint_by_position();

    /**
     * @brief Publish a trajectory setpoint by VELOCITY
     */
    void publish_trajectory_setpoint_by_velocity();


    /* ---- Timers callback ---- */ 

    /**
     * @brief Send heartbeat signal to keep connection
     */
    void send_heartbeat_signal();


    /* ---- Service server callback ---- */ 

    /** 
     * @brief Enable trajectory setpoint publishing
     */
    void enable_trajectory_setpoint_publishing_service_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        
        
    /** 
        * @brief Arm the vehicle
        */
    void arm_service_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /** 
        * @brief Disarm the vehicle
        */
    void disarm_service_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /** 
     * @brief Engage Offboard mode
     */
    void engage_offboard_mode_service_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /** 
     * @brief Engage Land mode
     */
    void engage_land_mode_service_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

};


#endif // PX4_COMMANDER_SERVER_NODE__HPP