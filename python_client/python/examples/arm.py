import rclpy
from px4_commander.api.python.px4_commander_client import Px4CommanderClient



class State(Enum):
    ARM = auto()            # Arm
    OFFBOARD = auto()       # Offboard
    DISARM = auto()         # Disarm
    LAND = auto()           # Land
    FINISHED = auto()       # Task finished