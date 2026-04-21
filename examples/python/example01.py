#!/usr/bin/env python3

"""
@brief Example state machine node: engage Offboard mode then arm the vehicle
@file example01.py

State flow:

  IDLE ──► ENGAGE_OFFBOARD ──► ARM ──► FLYING ──► ERROR (on any failure)

@author Thiago Marques de Oliveira
@date April, 2026
"""


# Private imports ---------------------------------------------------------

import rclpy
from rclpy.node import Node

from enum import Enum, auto

from px4_commander.px4_commander_client import Px4CommanderClient


# OffboardArmNode class ---------------------------------------------------

class OffboardArmNode(Node):

    # --------------------------------------------
    #   STATE MACHINE
    # --------------------------------------------

    class State(Enum):
        IDLE             = auto()
        ENGAGE_OFFBOARD  = auto()
        ARM              = auto()
        FLYING           = auto()
        ERROR            = auto()


    # --------------------------------------------
    #   CONSTRUCTOR
    # --------------------------------------------

    def __init__(self) -> None:
        super().__init__("offboard_arm_example_node")

        self._state     = self.State.IDLE
        self._commander = Px4CommanderClient()

        # Tick the state machine at 1 Hz
        self._timer = self.create_timer(1.0, self._tick)

        self.get_logger().info("Node started — initial state: IDLE")


    # --------------------------------------------
    #   STATE MACHINE TICK
    # --------------------------------------------

    def _tick(self) -> None:
        """Advance the state machine by one step."""

        dispatch = {
            self.State.IDLE:            self._on_idle,
            self.State.ENGAGE_OFFBOARD: self._on_engage_offboard,
            self.State.ARM:             self._on_arm,
            self.State.FLYING:          self._on_flying,
            self.State.ERROR:           self._on_error,
        }

        dispatch[self._state]()


    # --------------------------------------------
    #   STATE HANDLERS
    # --------------------------------------------

    def _on_idle(self) -> None:
        """IDLE — entry point, immediately transitions to ENGAGE_OFFBOARD."""
        self.get_logger().info("[IDLE] Starting sequence...")
        self._transition_to(self.State.ENGAGE_OFFBOARD)

    def _on_engage_offboard(self) -> None:
        """ENGAGE_OFFBOARD — send offboard mode command and wait for confirmation."""
        self.get_logger().info("[ENGAGE_OFFBOARD] Engaging offboard mode...")

        success, message = self._commander.engage_offboard_mode()

        if success:
            self.get_logger().info(f"[ENGAGE_OFFBOARD] {message}")
            self._transition_to(self.State.ARM)
        else:
            self.get_logger().error(f"[ENGAGE_OFFBOARD] {message}")
            self._transition_to(self.State.ERROR)

    def _on_arm(self) -> None:
        """ARM — arm the vehicle after offboard mode is confirmed."""
        self.get_logger().info("[ARM] Arming vehicle...")

        success, message = self._commander.arm()

        if success:
            self.get_logger().info(f"[ARM] {message}")
            self._transition_to(self.State.FLYING)
        else:
            self.get_logger().error(f"[ARM] {message}")
            self._transition_to(self.State.ERROR)

    def _on_flying(self) -> None:
        """FLYING — vehicle is armed and in offboard mode, ready for control."""
        self.get_logger().info("[FLYING] Vehicle is flying in offboard mode")

        # Stop the tick timer — nothing else to do in this example
        self._timer.cancel()

    def _on_error(self) -> None:
        """ERROR — unrecoverable failure, shut down the node."""
        self.get_logger().error("[ERROR] Sequence failed — shutting down")
        self._timer.cancel()
        rclpy.shutdown()


    # --------------------------------------------
    #   HELPERS
    # --------------------------------------------

    def _transition_to(self, next_state: "OffboardArmNode.State") -> None:
        """Log and perform a state transition."""
        self.get_logger().info(
            f"State transition: {self._state.name} -> {next_state.name}"
        )
        self._state = next_state


# Node initialization -----------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    node = OffboardArmNode()

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