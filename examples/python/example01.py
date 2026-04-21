#!/usr/bin/env python3

"""
Brief: Example state machine node: engage Offboard mode then arm the vehicle
File: example01.py

Author: Thiago Marques de Oliveira
Date: April, 2026
"""

"""
State flow:
  IDLE -> ENGAGE_OFFBOARD -> ARM -> FINISHED
"""


# Imports -----------------------------------------------------------------

# ROS 2 Core
import rclpy
from rclpy.node import Node

# Enum for state machine
from enum import Enum, auto

# PX4 Commander
from px4_commander.px4_commander_client import Px4CommanderClient


# State Machine class -----------------------------------------------------

class StateMachineNode(Node):

    # --------------------------------------------
    #   STATE MACHINE
    # --------------------------------------------

    class State(Enum):
        IDLE             = auto()
        ENGAGE_OFFBOARD  = auto()
        ARM              = auto()
        FINISHED         = auto()


    # --------------------------------------------
    #   CONSTRUCTOR
    # --------------------------------------------

    def __init__(self) -> None:
        super().__init__("offboard_and_arm_node")

        # Initial state
        self.state_     = self.State.IDLE

        # Init PX4 commander
        self.px4_commander_ = Px4CommanderClient()

        # Tick the state machine at 1 Hz
        self._timer = self.create_timer(1.0, self.state_machine_loop)

        self.get_logger().info("Node started — initial state: IDLE")


    # --------------------------------------------
    #   STATE MACHINE LOOP
    # --------------------------------------------

    def state_machine_loop(self) -> None:
        """
        Advance the state machine by one step.
        """

        dispatch = {
            self.State.IDLE:            self._on_idle,
            self.State.ENGAGE_OFFBOARD: self._on_engage_offboard,
            self.State.ARM:             self._on_arm,
            self.State.FINISHED:        self._on_finished,
        }

        dispatch[self.state_]()


    # --------------------------------------------
    #   STATE HANDLERS
    # --------------------------------------------

    def _on_idle(self) -> None:
        """
        IDLE - entry point, immediately transitions to ENGAGE_OFFBOARD.
        """

        self.px4_commander_.enable_trajectory_setpoint()

        self.get_logger().info("[IDLE] Starting sequence...")
        self._transition_to(self.State.ENGAGE_OFFBOARD)


    def _on_engage_offboard(self) -> None:
        """
        ENGAGE_OFFBOARD - send offboard mode command and wait for confirmation.
        """
        self.get_logger().info("[ENGAGE_OFFBOARD] Engaging offboard mode...")

        success, message = self.px4_commander_.engage_offboard_mode()

        if success:
            self.get_logger().info(f"[ENGAGE_OFFBOARD] {message}")
            self._transition_to(self.State.ARM)
        else:
            self.get_logger().error(f"[ENGAGE_OFFBOARD] {message}")
            self._transition_to(self.State.ERROR)


    def _on_arm(self) -> None:
        """
        ARM - arm the vehicle after offboard mode is confirmed.
        """
        self.get_logger().info("[ARM] Arming vehicle...")

        success, message = self.px4_commander_.arm()

        if success:
            self.get_logger().info(f"[ARM] {message}")
            self._transition_to(self.State.FINISHED)
        else:
            self.get_logger().error(f"[ARM] {message}")
            self._transition_to(self.State.ERROR)


    def _on_finished(self) -> None:
        """
        FINISHED - vehicle is armed and in offboard mode, ready for control.
        """
        self.get_logger().info("[FINISHED] Mission has finished")

        # Stop the tick timer — nothing else to do in this example
        self._timer.cancel()


    def _on_error(self) -> None:
        """
        ERROR - unrecoverable failure, shut down the node.
        """
        self.get_logger().error("[ERROR] Sequence failed — shutting down")
        self._timer.cancel()
        rclpy.shutdown()


    # --------------------------------------------
    #   HELPERS
    # --------------------------------------------

    def _transition_to(self, next_state: "StateMachineNode.State") -> None:
        """
        Log and perform a state transition.
        """
        self.get_logger().info(
            f"State transition: {self.state_.name} -> {next_state.name}"
        )
        self.state_ = next_state


# Node initialization -----------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    node = StateMachineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._commander.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()