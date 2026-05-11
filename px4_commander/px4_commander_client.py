# px4_commander_client.py

"""
PX4 commander client for calling arming and navigation services

Author : Thiago Marques de Oliveira
Date   : April, 2026
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool


# Structs ------------------------------------------------------ #

@dataclass
class CommandResult:
    """Result returned by every commander client call."""
    success: bool
    message: str

    def __iter__(self):
        """Make the result unpackable as a tuple: success, message = commander.arm()"""
        yield self.success
        yield self.message

    def __repr__(self) -> str:
        """Return a human-readable string representation"""
        status = "OK" if self.success else "FAIL"
        return f"CommandResult({status}: {self.message!r})"


# ---------------------------------------------------------------------------
#   Main client
# ---------------------------------------------------------------------------

class Px4CommanderClient:
    """
    Python interface for the services exposed by the Px4CommanderNode.

    Parameters
    ----------
    namespace : str
        ROS 2 namespace prefix of the commander node (default: "px4_commander").
    timeout_sec : float
        Time (seconds) to wait for each service to become available.
    node_name : str
        Name of the ROS 2 node created by this client.
    """

    # Services name (mirroring those defined in Px4CommanderNode)
    _SVC_ARM                         = "arm"
    _SVC_DISARM                      = "disarm"
    _SVC_ENGAGE_OFFBOARD_MODE        = "engage_offboard_mode"
    _SVC_ENGAGE_LAND_MODE            = "engage_land_mode"
    _SVC_ENGAGE_TAKEOFF_MODE         = "engage_takeoff_mode"
    _SVC_PUBLISH_TRAJECTORY_SETPOINT = "enable_trajectory_setpoint_publishing"


    # --------------------------------------------
    #   CONSTRUCTOR
    # --------------------------------------------

    def __init__(
        self,
        node_name = "px4_commander_client_node",
        namespace: str = "px4_commander",
        timeout_sec: float = 5.0,
    ) -> None:

        self.timeout_sec_ = timeout_sec
        self.ns_ = namespace

        # Create a dedicated internal node to own the service clients.
        # This prevents the caller's node from being added to a second executor
        # when spin_until_future_complete is called inside call().
        self.node_ = rclpy.create_node(node_name)
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self.node_)

        # Create all clients
        self.cli_arm_      = self._create_client(self._SVC_ARM)
        self.cli_disarm_   = self._create_client(self._SVC_DISARM)
        self.cli_offboard_ = self._create_client(self._SVC_ENGAGE_OFFBOARD_MODE)
        self.cli_land_     = self._create_client(self._SVC_ENGAGE_LAND_MODE)
        self.cli_takeoff_  = self._create_client(self._SVC_ENGAGE_TAKEOFF_MODE)
        self.cli_setpoint_ = self._create_client(self._SVC_PUBLISH_TRAJECTORY_SETPOINT)


    # --------------------------------------------
    #   INNER HELPERS
    # --------------------------------------------

    def _full_name(self, service_name: str) -> str:
        """
        Build the fully-qualified service name with optional namespace prefix.
        """
        return f"/{self.ns_}/{service_name}" if self.ns_ else service_name


    def _create_client(self, service_name: str):
        """
        Create a SetBool service client on the internal client node.
        """
        cli = self.node_.create_client(SetBool, self._full_name(service_name))
        return cli


    def _wait_for_service(self, cli, service_name: str) -> bool:
        """
        Bloqueia até o serviço estar disponível ou timeout.
        """
        if not cli.wait_for_service(timeout_sec=self.timeout_sec_):
            self.node_.get_logger().error(
                f"Service '{self._full_name(service_name)}' unavailable"
            )
            return False
        return True

    def _call(self, cli, service_name: str, data: bool) -> CommandResult:
        """
        Send a SetBool request and block until response or timeout.

        Parameters
        ----------
        cli          : ROS 2 service client
        service_name : readable name (for error logs)
        data         : value of the 'data' field in the request
        """

        # Wait for service to become available
        if not self._wait_for_service(cli, service_name):
            return CommandResult(
                success=False,
                message=f"Service '{service_name}' unavailable",
            )

        # Build and send request
        req = SetBool.Request()
        req.data = data

        future = cli.call_async(req)

        # Spin only the internal client node — the caller's node is never touched
        self.executor_.spin_until_future_complete(future, timeout_sec=self.timeout_sec_)

        if future.result() is None:
            return CommandResult(
                success=False,
                message=f"No response from service '{service_name}' (timeout)",
            )

        resp = future.result()
        return CommandResult(success=resp.success, message=resp.message)

    # --------------------------------------------
    #   PUBLIC API
    # --------------------------------------------

    def arm(self) -> CommandResult:
        """
        Arm the vehicle.
        """
        self.node_.get_logger().info("Requesting ARM...")
        return self._call(self.cli_arm_, self._SVC_ARM, data=True)


    def disarm(self) -> CommandResult:
        """
        Disarm the vehicle.
        """
        self.node_.get_logger().info("Requesting DISARM...")
        return self._call(self.cli_disarm_, self._SVC_DISARM, data=True)


    def engage_offboard_mode(self) -> CommandResult:
        """
        Engage Offboard mode.
        """
        self.node_.get_logger().info("Requesting OFFBOARD mode...")
        return self._call(self.cli_offboard_, self._SVC_ENGAGE_OFFBOARD_MODE, data=True)


    def engage_land_mode(self) -> CommandResult:
        """
        Engage automatic landing mode (AUTO_LAND).
        """
        self.node_.get_logger().info("Requesting LAND mode...")
        return self._call(self.cli_land_, self._SVC_ENGAGE_LAND_MODE, data=True)


    def engage_takeoff_mode(self) -> CommandResult:
        """
        Engage automatic takeoff mode (AUTO_TAKEOFF).
        """
        self.node_.get_logger().info("Requesting TAKEOFF mode...")
        return self._call(self.cli_takeoff_, self._SVC_ENGAGE_TAKEOFF_MODE, data=True)


    def enable_trajectory_setpoint(self) -> CommandResult:
        """
        Enable continuous trajectory setpoint publishing.
        """
        self.node_.get_logger().info("Enabling trajectory setpoint...")
        return self._call(self.cli_setpoint_, self._SVC_PUBLISH_TRAJECTORY_SETPOINT, data=True)


    def disable_trajectory_setpoint(self) -> CommandResult:
        """
        Disable continuous trajectory setpoint publishing.
        """
        self.node_.get_logger().info("Disabling trajectory setpoint...")
        return self._call(self.cli_setpoint_, self._SVC_PUBLISH_TRAJECTORY_SETPOINT, data=False)


    def destroy(self) -> None:
        """Release the ROS 2 node and executor."""
        self.node_.destroy_node()


# ---------------------------------------------------------------------------
#   Example usage (python px4_commander_client.py)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    rclpy.init()
    commander = Px4CommanderClient(timeout_sec=5.0)

    steps = [
        ("arm",                  commander.arm),
        ("engage_offboard_mode", commander.engage_offboard_mode),
        ("disarm",               commander.disarm),
        ("engage_land_mode",     commander.engage_land_mode),
    ]

    for label, fn in steps:
        result = fn()
        status = "ok" if result.success else "fail"
        print(f"[{status}] {label}: {result.message}")
        if not result.success:
            print(f"    Abortando sequência.")
            break

    commander.destroy()
    rclpy.shutdown()