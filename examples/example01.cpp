/****************************************************************************
 *
 * Copyright 2026 Thiago Marques de Oliveira
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
 * @brief Example state machine node: engage Offboard mode then arm the vehicle
 * @file offboard_arm_example.cpp
 *
 * State flow:
 *
 *   IDLE ──► ENGAGE_OFFBOARD ──► ARM ──► FLYING ──► ERROR (on any failure)
 *
 * @author Thiago Marques de Oliveira
 * @date April, 2026
 */


/* Private includes ------------------------------------------------------ */

#include "px4_commander/px4_commander_client.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <string>


/* Namespaces ------------------------------------------------------------ */

using namespace std::chrono_literals;


/* OffboardArmNode class ------------------------------------------------- */

class OffboardArmNode : public rclcpp::Node
{
public:

    // --------------------------------------------
    //   STATE MACHINE
    // --------------------------------------------

    enum class State
    {
        IDLE,
        ENGAGE_OFFBOARD,
        ARM,
        FLYING,
        ERROR
    };


    // --------------------------------------------
    //   CONSTRUCTOR
    // --------------------------------------------

    OffboardArmNode()
    : Node("offboard_arm_example_node"),
      state_(State::IDLE)
    {
        commander_ = std::make_shared<Px4CommanderClient>(
            this->shared_from_this()
        );

        // Tick the state machine at 1 Hz
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&OffboardArmNode::tick, this)
        );

        RCLCPP_INFO(this->get_logger(), "Node started — initial state: IDLE");
    }


private:

    // --------------------------------------------
    //   STATE MACHINE TICK
    // --------------------------------------------

    /**
     * @brief Advance the state machine by one step
     */
    void tick()
    {
        switch (state_)
        {
            case State::IDLE:
                on_idle();
                break;

            case State::ENGAGE_OFFBOARD:
                on_engage_offboard();
                break;

            case State::ARM:
                on_arm();
                break;

            case State::FLYING:
                on_flying();
                break;

            case State::ERROR:
                on_error();
                break;
        }
    }


    // --------------------------------------------
    //   STATE HANDLERS
    // --------------------------------------------

    /**
     * @brief IDLE — entry point, immediately transitions to ENGAGE_OFFBOARD
     */
    void on_idle()
    {
        RCLCPP_INFO(this->get_logger(), "[IDLE] Starting sequence...");
        transition_to(State::ENGAGE_OFFBOARD);
    }

    /**
     * @brief ENGAGE_OFFBOARD — send offboard mode command and wait for confirmation
     */
    void on_engage_offboard()
    {
        RCLCPP_INFO(this->get_logger(), "[ENGAGE_OFFBOARD] Engaging offboard mode...");

        const auto [success, message] = commander_->engage_offboard_mode();

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "[ENGAGE_OFFBOARD] %s", message.c_str());
            transition_to(State::ARM);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[ENGAGE_OFFBOARD] %s", message.c_str());
            transition_to(State::ERROR);
        }
    }

    /**
     * @brief ARM — arm the vehicle after offboard mode is confirmed
     */
    void on_arm()
    {
        RCLCPP_INFO(this->get_logger(), "[ARM] Arming vehicle...");

        const auto [success, message] = commander_->arm();

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "[ARM] %s", message.c_str());
            transition_to(State::FLYING);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[ARM] %s", message.c_str());
            transition_to(State::ERROR);
        }
    }

    /**
     * @brief FLYING — vehicle is armed and in offboard mode, ready for control
     */
    void on_flying()
    {
        RCLCPP_INFO(this->get_logger(), "[FLYING] Vehicle is flying in offboard mode");

        // Stop the tick timer — nothing else to do in this example
        timer_->cancel();
    }

    /**
     * @brief ERROR — unrecoverable failure, shut down the node
     */
    void on_error()
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Sequence failed — shutting down");
        timer_->cancel();
        rclcpp::shutdown();
    }


    // --------------------------------------------
    //   HELPERS
    // --------------------------------------------

    /**
     * @brief Log and perform a state transition
     * @param next  Target state
     */
    void transition_to(State next)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "State transition: %s → %s",
            state_to_string(state_),
            state_to_string(next)
        );

        state_ = next;
    }

    /**
     * @brief Convert a State enum value to a human-readable string
     * @param state  State to convert
     * @return const char* with the state name
     */
    static const char* state_to_string(State state)
    {
        switch (state)
        {
            case State::IDLE:             return "IDLE";
            case State::ENGAGE_OFFBOARD:  return "ENGAGE_OFFBOARD";
            case State::ARM:              return "ARM";
            case State::FLYING:           return "FLYING";
            case State::ERROR:            return "ERROR";
            default:                      return "UNKNOWN";
        }
    }


    // --------------------------------------------
    //   PRIVATE MEMBERS
    // --------------------------------------------

    State                                state_;
    std::shared_ptr<Px4CommanderClient>  commander_;
    rclcpp::TimerBase::SharedPtr         timer_;
};


// --------------------------------------------
//   NODE INITIALIZATION
// --------------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OffboardArmNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}