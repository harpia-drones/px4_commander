/**
 * @brief Example state machine node: engage Offboard mode then arm the vehicle
 * @file example01.cpp
 *
 * @author Thiago Marques de Oliveira
 * @date April, 2026
 */

/**
 * State flow:
 *   IDLE -> ENGAGE_OFFBOARD -> ARM -> FINISHED
 */


/* Includes -------------------------------------------------------------- */

#include <chrono>
#include <functional>
#include <string>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>

// PX4 Commander
#include "px4_commander/px4_commander_client.hpp"


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
        FINISHED
    };


    // --------------------------------------------
    //   CONSTRUCTOR
    // --------------------------------------------

    OffboardArmNode()
    : Node("offboard_arm_example_node"),
      state_(State::IDLE)
    {
        // NOTE: commander_ is intentionally NOT initialized here.
        // shared_from_this() is only valid after make_shared() completes,
        // i.e. after the constructor returns. Call init() from main().

        // Tick the state machine at 1 Hz
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&OffboardArmNode::state_machine_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Node started — initial state: IDLE");
    }


    // --------------------------------------------
    //   POST-CONSTRUCTION INITIALIZATION
    // --------------------------------------------

    /**
     * @brief Initialize members that require shared_from_this().
     *
     * Must be called from main() immediately after make_shared<OffboardArmNode>(),
     * once the shared_ptr is fully constructed.
     */
    void init()
    {
        commander_ = std::make_shared<Px4CommanderClient>(
            this->shared_from_this()
        );
    }


private:

    // --------------------------------------------
    //   STATE MACHINE LOOP
    // --------------------------------------------

    /**
     * @brief Advance the state machine by one step
     */
    void state_machine_loop()
    {
        switch (state_)
        {
            case State::IDLE:
                this->_on_idle();
                break;

            case State::ENGAGE_OFFBOARD:
                this->_on_engage_offboard();
                break;

            case State::ARM:
                this->_on_arm();
                break;

            case State::FINISHED:
                this->_on_finished();
                break;

            case State::ERROR:
                this->_on_error();
                break;
        }
    }


    // --------------------------------------------
    //   STATE HANDLERS
    // --------------------------------------------

    /**
     * @brief IDLE - entry point, immediately transitions to ENGAGE_OFFBOARD
     */
    void _on_idle()
    {
        RCLCPP_INFO(this->get_logger(), "[IDLE] Starting sequence...");
        this->_transition_to(State::ENGAGE_OFFBOARD);
    }

    /**
     * @brief ENGAGE_OFFBOARD - send offboard mode command and wait for confirmation
     */
    void _on_engage_offboard()
    {
        RCLCPP_INFO(this->get_logger(), "[ENGAGE_OFFBOARD] Engaging offboard mode...");

        const auto [success, message] = commander_->engage_offboard_mode();

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "[ENGAGE_OFFBOARD] %s", message.c_str());
            this->_transition_to(State::ARM);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[ENGAGE_OFFBOARD] %s", message.c_str());
            this->_transition_to(State::ERROR);
        }
    }

    /**
     * @brief ARM - arm the vehicle after offboard mode is confirmed
     */
    void _on_arm()
    {
        RCLCPP_INFO(this->get_logger(), "[ARM] Arming vehicle...");

        const auto [success, message] = commander_->arm();

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "[ARM] %s", message.c_str());
            this->_transition_to(State::FINISHED);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[ARM] %s", message.c_str());
            this->_transition_to(State::ERROR);
        }
    }

    /**
     * @brief FINISHED - vehicle is armed and in offboard mode, ready for control
     */
    void _on_finished()
    {
        RCLCPP_INFO(this->get_logger(), "[FINISHED] Mission has finished");

        // Stop the tick timer — nothing else to do in this example
        this->timer_->cancel();
    }

    /**
     * @brief ERROR - unrecoverable failure, shut down the node
     */
    void _on_error()
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Sequence failed — shutting down");
        this->timer_->cancel();
        rclcpp::shutdown();
    }


    // --------------------------------------------
    //   HELPERS
    // --------------------------------------------

    /**
     * @brief Log and perform a state transition
     * @param next  Target state
     */
    void _transition_to(State next)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "State transition: %s -> %s",
            state_to_string(this->state_),
            state_to_string(next)
        );

        this->state_ = next;
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
            case State::FINISHED:         return "FINISHED";
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

    // make_shared must complete before init() is called,
    // so that shared_from_this() inside init() is valid
    auto node = std::make_shared<OffboardArmNode>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}