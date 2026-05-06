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
 * @file px4_commander_server_node.cpp
 *
 * @author Thiago Marques de Oliveira
 * @date April, 2026
 */



/* Private includes ------------------------------------------------------ */

#include "px4_commander/px4_commander_server_node.hpp"


/* Namespaces ------------------------------------------------------------ */

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace px4_msgs::msg;


/* Px4CommanderServerNode class ------------------------------------------ */


// --------------------------------------------
//   CONSTRUCTOR
// --------------------------------------------


Px4CommanderServerNode::Px4CommanderServerNode(const std::string node_name) 
: Node(node_name)
{
    // --------------------------------------------
    //   PARAMETERS
    // --------------------------------------------

    /* ---- Declare parameters ---- */ 

    this->declare_parameter("response_timeout", 5);
    this->declare_parameter("pre_offboard_waiting_time", 2.0);
    this->declare_parameter("is_simulation", true);

    /* ---- Get parameters ---- */ 

    long int response_timeout_int;
    double pre_offboard_waiting_time_double;

    this->get_parameter("response_timeout", response_timeout_int);
    this->get_parameter("pre_offboard_waiting_time", pre_offboard_waiting_time_double);
    this->get_parameter("is_simulation", this->is_simulation_);

    cv_response_timeout_ =  std::chrono::seconds(response_timeout_int);
    pre_offboard_waiting_time_ = rclcpp::Duration::from_seconds(pre_offboard_waiting_time_double);

    
    // --------------------------------------------
    //   CALLBACK GROUP
    // --------------------------------------------

    /* ---- Groups ---- */ 
    control_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    /* ---- Subscription group config ---- */ 
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = control_group_;

    
    // --------------------------------------------
    //   TIMERS
    // --------------------------------------------

    send_heartbeat_signal_timer_ = this->create_wall_timer(
        100ms, 
        std::bind(&Px4CommanderServerNode::send_heartbeat_signal, this),
        control_group_
    );


    // --------------------------------------------
    //   PUBLISHERS
    // --------------------------------------------

    // Quality of Service
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                    .best_effort()
                    .durability_volatile();

    // Offboard control mode publisher
    this->offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);

    // Trajectory setpoint publisher
    this->trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);

    // Vechicle command publisher
    this->vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    
    // --------------------------------------------
    //   SUBSCRIPTIONS
    // --------------------------------------------

    // Vehicle status subscription
    this->vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
            this->is_simulation_ ? "/fmu/out/vehicle_status" : "/fmu/out/vehicle_status_v1", 
            qos,
            [this](const VehicleStatus::ConstSharedPtr vehicle_status)
            {
                this->current_arming_state_.store(vehicle_status->arming_state, std::memory_order_relaxed);
                this->current_nav_state_.store(vehicle_status->nav_state, std::memory_order_relaxed);
                this->timestamp_.store(vehicle_status->timestamp, std::memory_order_relaxed);

                wait_success_cv_.notify_all();
            },
            sub_opts
        );


    // --------------------------------------------
    //   SERVICE SERVERS
    // --------------------------------------------

    this->arm_service_ = this->create_service<std_srvs::srv::SetBool>(
        "arm",
        std::bind(&Px4CommanderServerNode::arm_service_callback, this, _1, _2),
        rclcpp::QoS(rclcpp::ServicesQoS()),
        service_group_
    );  

    this->disarm_service_ = this->create_service<std_srvs::srv::SetBool>(
        "disarm",
        std::bind(&Px4CommanderServerNode::disarm_service_callback, this, _1, _2),
        rclcpp::QoS(rclcpp::ServicesQoS()),
        service_group_
    );

    this->engage_land_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "engage_land_mode",
        std::bind(&Px4CommanderServerNode::engage_land_mode_service_callback, this, _1, _2),
        rclcpp::QoS(rclcpp::ServicesQoS()),
        service_group_
    );

    this->engage_offboard_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "engage_offboard_mode",
        std::bind(&Px4CommanderServerNode::engage_offboard_mode_service_callback, this, _1, _2),
        rclcpp::QoS(rclcpp::ServicesQoS()),
        service_group_
    );

    this->enable_trajectory_setpoint_publishing_service_ = this->create_service<std_srvs::srv::SetBool>(
        "enable_trajectory_setpoint_publishing",
        std::bind(&Px4CommanderServerNode::enable_trajectory_setpoint_publishing_service_callback, this, _1, _2),
        rclcpp::QoS(rclcpp::ServicesQoS()),
        service_group_
    );


    RCLCPP_INFO(this->get_logger(), "PX4 Commander node has started");
}



// --------------------------------------------
//   PX4 METHODS (Publishers and Subscribers)
// --------------------------------------------


/**
 * @brief Publish vehicle commands
 * @param args VehicleCommand struct with command and parameters 1-7
 */
void Px4CommanderServerNode::publish_vehicle_command(const VehicleCommandArgs& args)
{
    VehicleCommand msg{};
    msg.param1 = args.param1.value_or(0.0f);
    msg.param2 = args.param2.value_or(0.0f);
    msg.param3 = args.param3.value_or(0.0f);
    msg.param4 = args.param4.value_or(0.0f);
    msg.param5 = args.param5.value_or(0.0f);
    msg.param6 = args.param6.value_or(0.0f);
    msg.param7 = args.param7.value_or(0.0f);
    msg.command = args.command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->timestamp_.load(std::memory_order_relaxed);
    vehicle_command_publisher_->publish(msg);
}


/**
 * @brief Send a command to arm the vehicle
 */
void Px4CommanderServerNode::arm()
{
	this->publish_vehicle_command({
        .command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        .param1 = VehicleCommand::ARMING_ACTION_ARM
    });

	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}


/**
 * @brief Send a command to disarm the vehicle
 */
void Px4CommanderServerNode::disarm()
{
	this->publish_vehicle_command({
        .command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        .param1 = VehicleCommand::ARMING_ACTION_DISARM
    });

	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}


/**
 * @brief Send a command to land the vehicle
 */
void Px4CommanderServerNode::land()
{
    this->publish_vehicle_command({
        .command = VehicleCommand::VEHICLE_CMD_NAV_LAND
    });

	RCLCPP_INFO(this->get_logger(), "Auto Land command sent");
}


/**
 * @brief Send a command to engage offboard mode
 */
void Px4CommanderServerNode::engage_offboard_mode()
{
    this->publish_vehicle_command({
        .command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        .param1 = 1.0f, // Custom mode
        .param2 = px4_custom_mode::PX4_CUSTOM_MAIN_MODE_OFFBOARD
    });

	RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}


/**
 * @brief Publish the offboard control mode to be controlled by POSITION
 */
void Px4CommanderServerNode::publish_offboard_control_mode_by_position()
{
    OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->timestamp_.load(std::memory_order_relaxed);
	this->offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish the offboard control mode to be controlled by VELOCITY
 */
void Px4CommanderServerNode::publish_offboard_control_mode_by_velocity()
{
    OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->timestamp_.load(std::memory_order_relaxed);
	this->offboard_control_mode_publisher_->publish(msg);
}


/**
* @brief Publish a trajectory setpoint by POSITION
*/
void Px4CommanderServerNode::publish_trajectory_setpoint_by_position()
{
    TrajectorySetpoint msg{};
    msg.position = {0.0f, 0.0f, -1.0f};
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.yaw = 0.0f;  // [-pi,pi]
    msg.yawspeed = NAN;
    msg.timestamp = this->timestamp_.load(std::memory_order_relaxed);
    this->trajectory_setpoint_publisher_->publish(msg);
}


/**
* @brief Publish a trajectory setpoint by VELOCITY
*/
void Px4CommanderServerNode::publish_trajectory_setpoint_by_velocity()
{
    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {0.0f, 0.0f, 0.0f};
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.yaw = NAN;  // [-pi,pi]
    msg.yawspeed = 0.0f;
    msg.timestamp = this->timestamp_.load(std::memory_order_relaxed);
    this->trajectory_setpoint_publisher_->publish(msg);
}



// --------------------------------------------
//   SERVICES
// --------------------------------------------


/** 
* @brief Menage trajectory setpoint publishing
*/
void Px4CommanderServerNode::enable_trajectory_setpoint_publishing_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    bool requested_publish_trajectory_setpoint = request->data;
    
    if (requested_publish_trajectory_setpoint)
    {
        publish_trajectory_setpoint_.store(true, std::memory_order_relaxed);
        response->success = true;
        response->message = "Trajectory setpoint publishing started";
    }
    else
    {
        publish_trajectory_setpoint_.store(false, std::memory_order_relaxed);
        response->success = true;
        response->message = "Trajectory setpoint publishing stopped";
    }
}


/** 
* @brief Arm the vehicle
*/
void Px4CommanderServerNode::arm_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    bool arm_requested = request->data;

    if (arm_requested)
    {
        // Arm
        this->arm();

        // Maximum time for success after sending command
        const auto timeout = std::chrono::steady_clock::now() + this->cv_response_timeout_;

        std::unique_lock<std::mutex> lock(wait_success_mutex_);
        const bool ok = wait_success_cv_.wait_until(
            lock,
            timeout,
            [this]() {
                const auto arming = current_arming_state_.load(std::memory_order_relaxed);
                return (arming == VehicleStatus::ARMING_STATE_ARMED);
            }
        );

        if (ok) 
        {
            response->success = true;
            response->message = "Vehicle armed successfully";
            RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
        } 
        else 
        {
            response->success = false;
            response->message = "Fail to arm vehicle";
            RCLCPP_ERROR(this->get_logger(), "ERROR: Fail to arm vehicle");
        }
    }
}


/** 
* @brief Disarm the vehicle
*/
void Px4CommanderServerNode::disarm_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    bool disarm_requested = request->data;

    if (disarm_requested)
    {
        // Disarm
        this->disarm();

        // Maximum time for success after sending command
        const auto timeout = std::chrono::steady_clock::now() + this->cv_response_timeout_;

        std::unique_lock<std::mutex> lock(wait_success_mutex_);
        const bool ok = wait_success_cv_.wait_until(
            lock,
            timeout,
            [this]() {
                const auto arming = current_arming_state_.load(std::memory_order_relaxed);
                return (arming == VehicleStatus::ARMING_STATE_DISARMED);
            }
        );

        if (ok) 
        {
            response->success = true;
            response->message = "Vehicle disarmed successfully";
            RCLCPP_INFO(this->get_logger(), "Vehicle disarmed successfully");
        } 
        else 
        {
            response->success = false;
            response->message = "Fail to disarm vehicle";
            RCLCPP_ERROR(this->get_logger(), "Fail to disarm vehicle");
        }
    }
}


/** 
* @brief Engage Offboard mode
*/
void Px4CommanderServerNode::engage_offboard_mode_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    bool engage_offboard_mode_requested = request->data;
    
    if (engage_offboard_mode_requested)
    {
        // Publish offboard control mode and trajectory setpoint for 2 sec at 50 hz
        {
            rclcpp::Rate rate(50); // 50 Hz
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            
            while(std::chrono::steady_clock::now() < deadline)
            {
                this->publish_offboard_control_mode_by_position();
                this->publish_trajectory_setpoint_by_position();
                rate.sleep();
            }

            // Engage Offboard mode
            this->engage_offboard_mode();
        }

        // Maximum time for success after sending command
        const auto timeout = std::chrono::steady_clock::now() + this->cv_response_timeout_;

        std::unique_lock<std::mutex> lock(wait_success_mutex_);
        const bool ok = wait_success_cv_.wait_until(
            lock,
            timeout,
            [this]() {
                const auto nav = current_nav_state_.load(std::memory_order_relaxed);
                return (nav == VehicleStatus::NAVIGATION_STATE_OFFBOARD);
            }
        );

        if (ok) 
        {
            response->success = true;
            response->message = "Switched to OFFBOARD mode";
            RCLCPP_INFO(this->get_logger(), "Switched to OFFBOARD mode");
        } 
        else 
        {
            response->success = false;
            response->message = "Fail to engage OFFBOARD mode";
            RCLCPP_ERROR(this->get_logger(), "Fail to engage OFFBOARD mode");
        }
    }
}


/** 
* @brief Engage Land mode
*/
void Px4CommanderServerNode::engage_land_mode_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    bool engage_land_mode_requested = request->data;
    
    if (engage_land_mode_requested)
    {
        // Land
        this->land();

        // Maximum time for success after sending command
        const auto timeout = std::chrono::steady_clock::now() + this->cv_response_timeout_;

        std::unique_lock<std::mutex> lock(wait_success_mutex_);
        const bool ok = wait_success_cv_.wait_until(
            lock,
            timeout,
            [this]() {
                const auto nav = current_nav_state_.load(std::memory_order_relaxed);
                return (nav == VehicleStatus::NAVIGATION_STATE_AUTO_LAND);
            }
        );

        if (ok) 
        {
            response->success = true;
            response->message = "Switched to AUTO_LAND mode";
            RCLCPP_INFO(this->get_logger(), "Switched to AUTO_LAND mode");
        } 
        else 
        {
            response->success = false;
            response->message = "Fail to switch to AUTO_LAND mode";
            RCLCPP_ERROR(this->get_logger(), "Fail to switch to AUTO_LAND mode");
        }
    }
}



// --------------------------------------------
//   TIMERS
// --------------------------------------------


/**
 * @brief Send heartbeat signal to keep connection
 */
void Px4CommanderServerNode::send_heartbeat_signal()
{
    if (publish_trajectory_setpoint_.load(std::memory_order_relaxed))
    {   
        this->publish_offboard_control_mode_by_velocity();
        this->publish_trajectory_setpoint_by_velocity();
        // RCLCPP_WARN(this->get_logger(), "Publishing offboard control mode by velocity");
    }
    else
    {
        this->publish_offboard_control_mode_by_position();
        // RCLCPP_WARN(this->get_logger(), "Publishing offboard control mode by position");
    }
}



// --------------------------------------------
//   NODE INITIALIZATION
// --------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Px4CommanderServerNode>("px4_commander_server_node");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}