import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Importeer de interfaces uit jouw robot- en camera packages
# LET OP: Vervang 'my_robot_interfaces' en 'my_camera_interfaces' door de echte namen.
from my_robot_interfaces.msg import SystemState
from my_robot_interfaces.srv import CheckReachability, GoHome
from my_robot_interfaces.action import PickAndPlace

# Dummy import voor een AI-camera bericht (bijv. met objectklasse en x,y,z coördinaten)
from my_camera_interfaces.msg import DetectionArray

class CentralControllerUnit(Node):

    def __init__(self):
        super().__init__('central_controller')
        self.get_looger().info("De Centrale Controller Unit is gestart.")

    
        # ===================================
        # 1. Ontvangen informatie van camera en Voice Control Unit
        # ===================================
        self.camera_info = self.create_subscription(
            CameraInfo
            # Invoegen Camera service
        )

        self.voice_command = self.create_subscription(
            VoiceCommand
            # Invoegen voice control service
        )

        # ===================================
        # 2. Bepalen gewenste robotpositie
        # ===================================
        self.robot_goal_state = self.create_subscription(
            GoalState
            # Invoegen service die positie robot bepaald
        )

        self.drop_off_state = self.create_subscription(
            DropOffState
            # Invoegen service die locatie stortbak bepaald
        )


        # ===================================
        # 3. Controleren op errors
        # ===================================


        # ===================================
        # 4. Robot starten
        # ===================================

        # Action? met informatie over hoe de robot moet bewegen om de gewenste actie uit te voeren.
        def robot_controller(self, GoalState, DroppOffState):
                            


