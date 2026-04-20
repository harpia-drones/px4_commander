"""
px4_commander_client.py
=======================
Cliente Python para o nó PX4CommanderNode (ROS 2).

Autor : Thiago Marques de Oliveira
Data  : Abril, 2026

Uso básico
----------
    import rclpy
    from px4_commander_client import Px4CommanderClient

    rclpy.init()
    commander = Px4CommanderClient()

    success, message = commander.arm()
    success, message = commander.engage_offboard_mode()
    success, message = commander.disarm()
    success, message = commander.engage_land_mode()

    commander.destroy()
    rclpy.shutdown()
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool


# ---------------------------------------------------------------------------
#   Tipos de retorno
# ---------------------------------------------------------------------------

@dataclass
class CommandResult:
    """Resultado de um comando enviado ao PX4CommanderNode."""
    success: bool
    message: str

    def __iter__(self):
        """Permite desempacotar como tupla: success, message = commander.arm()"""
        yield self.success
        yield self.message

    def __repr__(self) -> str:
        status = "OK" if self.success else "FAIL"
        return f"CommandResult({status}: {self.message!r})"


# ---------------------------------------------------------------------------
#   Cliente principal
# ---------------------------------------------------------------------------

class Px4CommanderClient:
    """
    Interface Python para os serviços expostos pelo Px4CommanderNode.

    Parâmetros
    ----------
    namespace : str
        Prefixo ROS 2 do nó commander (padrão: '' — sem namespace).
    timeout_sec : float
        Tempo (segundos) para aguardar cada serviço ficar disponível.
    node_name : str
        Nome do nó ROS 2 criado por este cliente.
    """

    # Nomes dos serviços (espelham os definidos no Px4CommanderNode)
    _SVC_ARM                         = "arm"
    _SVC_DISARM                      = "disarm"
    _SVC_ENGAGE_OFFBOARD_MODE        = "engage_offboard_mode"
    _SVC_ENGAGE_LAND_MODE            = "engage_land_mode"
    _SVC_PUBLISH_TRAJECTORY_SETPOINT = "publish_trajectory_setpoint"

    def __init__(
        self,
        namespace: str = "",
        timeout_sec: float = 5.0,
        node_name: str = "px4_commander_client_node",
    ) -> None:

        self._timeout_sec = timeout_sec
        self._ns = namespace.rstrip("/")

        self._node = rclpy.create_node(node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        # Cria todos os clientes de serviço
        self._cli_arm = self._create_client(self._SVC_ARM)
        self._cli_disarm = self._create_client(self._SVC_DISARM)
        self._cli_offboard = self._create_client(self._SVC_ENGAGE_OFFBOARD_MODE)
        self._cli_land = self._create_client(self._SVC_ENGAGE_LAND_MODE)
        self._cli_setpoint = self._create_client(self._SVC_PUBLISH_TRAJECTORY_SETPOINT)

    # ------------------------------------------------------------------
    #   Helpers internos
    # ------------------------------------------------------------------

    def _full_name(self, service_name: str) -> str:
        return f"{self._ns}/{service_name}" if self._ns else service_name

    def _create_client(self, service_name: str):
        full = self._full_name(service_name)
        cli = self._node.create_client(SetBool, full)
        return cli

    def _wait_for_service(self, cli, service_name: str) -> bool:
        """Bloqueia até o serviço estar disponível ou timeout."""
        full = self._full_name(service_name)
        if not cli.wait_for_service(timeout_sec=self._timeout_sec):
            self._node.get_logger().error(
                f"Serviço '{full}' não disponível após {self._timeout_sec}s"
            )
            return False
        return True

    def _call(self, cli, service_name: str, data: bool) -> CommandResult:
        """
        Envia uma requisição SetBool e retorna um CommandResult.

        Parâmetros
        ----------
        cli         : cliente ROS 2 do serviço
        service_name: nome legível (para logs de erro)
        data        : valor do campo `data` na requisição
        """
        if not self._wait_for_service(cli, service_name):
            return CommandResult(
                success=False,
                message=f"Serviço '{service_name}' indisponível",
            )

        req = SetBool.Request()
        req.data = data

        future = cli.call_async(req)
        self._executor.spin_until_future_complete(future, timeout_sec=self._timeout_sec)

        if future.result() is None:
            return CommandResult(
                success=False,
                message=f"Sem resposta do serviço '{service_name}' (timeout)",
            )

        resp = future.result()
        return CommandResult(success=resp.success, message=resp.message)

    # ------------------------------------------------------------------
    #   API pública
    # ------------------------------------------------------------------

    def arm(self) -> CommandResult:
        """
        Armar o veículo.

        Retorna
        -------
        CommandResult
            .success  — True se o veículo foi armado com sucesso
            .message  — Mensagem descritiva do resultado
        """
        self._node.get_logger().info("Solicitando ARM...")
        return self._call(self._cli_arm, self._SVC_ARM, data=True)

    def disarm(self) -> CommandResult:
        """
        Desarmar o veículo.

        Retorna
        -------
        CommandResult
            .success  — True se o veículo foi desarmado com sucesso
            .message  — Mensagem descritiva do resultado
        """
        self._node.get_logger().info("Solicitando DISARM...")
        return self._call(self._cli_disarm, self._SVC_DISARM, data=True)

    def engage_offboard_mode(self) -> CommandResult:
        """
        Ativar o modo Offboard.

        O Px4CommanderNode publica offboard control mode + trajectory setpoint
        por ~2 s antes de enviar o comando, como exigido pelo PX4.

        Retorna
        -------
        CommandResult
            .success  — True se o modo Offboard foi ativado
            .message  — Mensagem descritiva do resultado
        """
        self._node.get_logger().info("Solicitando OFFBOARD mode...")
        return self._call(self._cli_offboard, self._SVC_ENGAGE_OFFBOARD_MODE, data=True)

    def engage_land_mode(self) -> CommandResult:
        """
        Ativar o modo de pouso automático (AUTO_LAND).

        Retorna
        -------
        CommandResult
            .success  — True se o modo AUTO_LAND foi ativado
            .message  — Mensagem descritiva do resultado
        """
        self._node.get_logger().info("Solicitando LAND mode...")
        return self._call(self._cli_land, self._SVC_ENGAGE_LAND_MODE, data=True)

    def set_trajectory_setpoint_publishing(self, enable: bool) -> CommandResult:
        """
        Habilitar ou desabilitar a publicação contínua de trajectory setpoints.

        Quando ativo, o nó publica offboard control mode por velocidade +
        trajectory setpoint por velocidade a cada 100 ms (heartbeat).

        Parâmetros
        ----------
        enable : bool
            True para iniciar, False para parar.

        Retorna
        -------
        CommandResult
            .success  — resultado da operação
            .message  — Mensagem descritiva do resultado
        """
        action = "ativando" if enable else "desativando"
        self._node.get_logger().info(f"{action} publicação de trajectory setpoint...")
        return self._call(self._cli_setpoint, self._SVC_PUBLISH_TRAJECTORY_SETPOINT, data=enable)

    def destroy(self) -> None:
        """Libera o nó ROS 2 e o executor."""
        self._node.destroy_node()


# ---------------------------------------------------------------------------
#   Exemplo de uso direto (python px4_commander_client.py)
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
        status = "✓" if result.success else "✗"
        print(f"[{status}] {label}: {result.message}")
        if not result.success:
            print(f"    Abortando sequência.")
            break

    commander.destroy()
    rclpy.shutdown()