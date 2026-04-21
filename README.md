# px4_commander

Pacote ROS 2 do sistema **Harpia** responsável por gerenciar os estados de arming e navegação do PX4. Ele expõe uma interface de serviços simples que abstrai toda a comunicação com o firmware, permitindo que outros pacotes do sistema — como o `movement_controller` — armem o veículo, troquem de modo de voo e iniciem o pouso sem precisar conhecer os detalhes do protocolo PX4.



## Visão Geral

O PX4 exige uma sequência específica para que comandos como arm e offboard mode sejam aceitos: o sistema externo precisa publicar continuamente mensagens de controle antes de enviar qualquer comando de modo, e manter esse fluxo ativo enquanto o drone estiver em voo. O `px4_commander` encapsula essa lógica internamente e expõe apenas serviços de alto nível para o restante do sistema.

O pacote é composto por dois componentes principais:

- **`Px4CommanderServerNode`** — nó ROS 2 em C++ que se comunica diretamente com o FMU via tópicos `px4_msgs`. Gerencia o heartbeat de offboard, executa os comandos e confirma o resultado monitorando o `VehicleStatus`.
- **`Px4CommanderClient`** — biblioteca cliente disponível em C++ e Python, para ser usada por outros nós do sistema que precisam acionar os serviços do commander sem reimplementar a lógica de chamada.

---

## Arquitetura

```
                       Harpia System
  ┌─────────────────────────────────────────────────────┐
  │                                                     │
  │        movement_controller         outro_no         │
  │               │                        │            │
  │               └──────────┬─────────────┘            │
  │                          │ ROS 2 Services           │
  │                          ▼                          │
  │              px4_commander_server_node              │
  │                          │                          │
  │               px4_msgs topics (uXRCE-DDS)           │
  │                          │                          │
  └──────────────────────────┼──────────────────────────┘
                             │
                            PX4
```

### Heartbeat e controle de setpoint

Enquanto o veículo está em modo Offboard, o PX4 exige que o sistema externo publique mensagens de controle continuamente — caso contrário, o firmware abandona o modo automaticamente. O `Px4CommanderServerNode` gerencia isso com um timer de 100 ms.

O comportamento do heartbeat depende do estado do flag `publish_trajectory_setpoint`:

| Estado do flag | O que é publicado a cada 100 ms |
|---|---|
| `true` (padrão) | `OffboardControlMode` por velocidade + `TrajectorySetpoint` zerado - mantém o link ativo sem mover o drone|
| `false` | Somente `OffboardControlMode` por posição - `TrajectorySetpoint` fica a encargo do `movement_controller` |

O `movement_controller` é responsável por desabilitar essa flag antes de enviar comandos de movimento e habilitá-la quando termina, devolvendo o controle do heartbeat ao commander.

### Sequência de ativação do modo Offboard

Antes de aceitar o comando de troca de modo, o PX4 exige que o sistema externo já esteja publicando mensagens de controle. Por isso, o serviço `engage_offboard_mode` publica offboard control mode e trajectory setpoint por aproximadamente 2 segundos a 50 Hz antes de enviar o comando, tudo isso internamente, sem que o chamador precise se preocupar.



## Instalação

```bash
git clone https://github.com/harpia-drones/px4_commander.git
```

### Dependências

- ROS 2 Jazzy
- `px4_msgs`
- `std_srvs`

### Build

```bash
cd ~/harpia_ws
colcon build --packages-select px4_commander
source install/setup.bash
```

## API

### Serviços expostos pelo `Px4CommanderServerNode`

| Serviço | Tipo | Descrição |
|---|---|---|
| `px4_commander/arm` | `std_srvs/SetBool` | Arma o veículo e aguarda confirmação |
| `px4_commander/disarm` | `std_srvs/SetBool` | Desarma o veículo e aguarda confirmação |
| `px4_commander/engage_offboard_mode` | `std_srvs/SetBool` | Ativa o modo Offboard |
| `px4_commander/engage_land_mode` | `std_srvs/SetBool` | Ativa o modo AUTO_LAND |
| `px4_commander/publish_trajectory_setpoint` | `std_srvs/SetBool` | Habilita (`true`) ou desabilita (`false`) a publicação de trajectory setpoints |

Todos os serviços retornam `success` (bool) e `message` (string) indicando o resultado da operação.

### Parâmetros do nó

| Parâmetro | Tipo | Padrão | Descrição |
|---|---|---|---|
| `response_timeout` | `int` | `5` | Tempo máximo (segundos) para aguardar confirmação do PX4 após um comando |
| `pre_offboard_waiting_time` | `double` | `2.0` | Tempo (segundos) de publicação prévia antes de engajar o modo Offboard |
| `simulation` | `bool` | `true` | Alterna o tópico de status entre simulação e hardware real |



## Uso

### Iniciando o nó

```bash
ros2 launch px4_commander px4_commander.launch.py
```

### Usando o cliente C++

```cpp
#include "px4_commander/px4_commander_client.hpp"

// No init() do seu nó (após make_shared):
px4_commander_client_ = std::make_shared<Px4CommanderClient>(this->shared_from_this());

// Armando o veículo:
const auto [success, message] = px4_commander_client_->arm();

// Engajando offboard e depois armando:
const auto [success, message] = px4_commander_client_->engage_offboard_mode();
if (success) px4_commander_client_->arm();

// Gerenciando o heartbeat de setpoint (ex: movement_controller):
px4_commander_client_->disable_trajectory_setpoint();  // antes de enviar movimento
px4_commander_client_->enable_trajectory_setpoint();   // ao terminar
```

No `CMakeLists.txt` do seu pacote:

```cmake
find_package(px4_commander REQUIRED)
ament_target_dependencies(meu_no px4_commander)
target_link_libraries(meu_no px4_commander_client)
```

### Usando o cliente Python

```python
from px4_commander.px4_commander_client import Px4CommanderClient

commander = Px4CommanderClient()

success, message = commander.arm()
success, message = commander.engage_offboard_mode()
success, message = commander.engage_land_mode()

# Ou acessando por atributo:
result = commander.arm()
if result.success:
    print(result.message)

commander.destroy()
```

### Exemplo completo

O pacote inclui o `example01` - uma máquina de estados que demonstra a sequência completa de ativação: engaja o modo Offboard e em seguida arma o veículo. 

```bash
ros2 run px4_commander example01.py   # Python
ros2 run px4_commander example01      # C++
```

Fluxo de estados:

```
IDLE ──► ENGAGE_OFFBOARD ──► ARM ──► FLYING
                │                      
                └──► ERROR (em qualquer falha)
```