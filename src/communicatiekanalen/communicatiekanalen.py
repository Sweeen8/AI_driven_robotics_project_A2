import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# LET OP: Vervang 'my_robot_interfaces' door de naam van jouw eigen package 
# waarin de custom messages, services en actions zijn gedefinieerd.
from my_robot_interfaces.msg import MotionStatus, CurrentPose, PlanningStatus, TaskResult, SystemState
from my_robot_interfaces.srv import GoHome, ResetError, CheckReachability
from my_robot_interfaces.action import PickAndPlace

class Motion(Node):

    def __init__(self):
        super().__init__('motion')
        self.get_logger().info("Motion node is gestart.")

        # ==========================================
        # PUBLISHERS (Node -> Centrale Controller/HMI)
        # ==========================================
        
        self.status_pub = self.create_publisher(
            MotionStatus,
            '/motion/status',
            10)
            
        self.current_pose_pub = self.create_publisher(
            CurrentPose,
            '/motion/current_pose',
            10)
            
        self.planning_status_pub = self.create_publisher(
            PlanningStatus,
            '/motion/planning_status',
            10)
            
        self.task_result_pub = self.create_publisher(
            TaskResult,
            '/motion/task_result',
            10)

        # ==========================================
        # SUBSCRIBERS (Centrale Controller -> Node)
        # ==========================================
        
        self.system_state_sub = self.create_subscription(
            SystemState,
            '/controller/system_state',
            self.system_state_callback,
            10)

        # ==========================================
        # SERVICE SERVERS (HMI/Centrale Controller -> Node)
        # ==========================================
        
        self.go_home_srv = self.create_service(
            GoHome,
            '/motion/go_home',
            self.go_home_callback)
            
        self.reset_error_srv = self.create_service(
            ResetError,
            '/motion/reset_error',
            self.reset_error_callback)
            
        self.check_reachability_srv = self.create_service(
            CheckReachability,
            '/motion/check_reachability',
            self.check_reachability_callback)

        # ==========================================
        # ACTION SERVERS (Centrale Controller -> Node)
        # ==========================================
        
        self.pick_and_place_action = ActionServer(
            self,
            PickAndPlace,
            '/motion/pick_and_place',
            self.pick_and_place_callback)

    # ==========================================
    # CALLBACK FUNCTIES
    # ==========================================

    def system_state_callback(self, msg):
        # Hier komt de logica om te controleren of motion_allowed TRUE is
        # en of er een noodstop actief is.
        pass

    def go_home_callback(self, request, response):
        self.get_logger().info('Commando ontvangen: go_home')
        # Logica om veilig naar home-pose te gaan
        response.success = True
        response.message = "Robot beweegt naar home"
        return response

    def reset_error_callback(self, request, response):
        self.get_logger().info('Commando ontvangen: reset_error')
        # Logica om errors te resetten
        response.success = True
        # response.resterende_error_code = ...
        return response

    def check_reachability_callback(self, request, response):
        self.get_logger().info('Commando ontvangen: check_reachability')
        # Logica voor path planning check zonder te bewegen
        response.reachable = True
        response.collision_free = True
        response.reason = "Geen obstakels gedetecteerd"
        return response

    def pick_and_place_callback(self, goal_handle):
        self.get_logger().info('Actie gestart: pick_and_place')
        
        # Voorbeeld van hoe je de goal waarden ophaalt:
        # task_id = goal_handle.request.task_id
        # target_bin = goal_handle.request.target_bin
        
        # TODO: Implementeer de werkelijke pick-and-place cyclus (meerdere seconden)
        # TODO: Publiceer tussentijdse feedback (motion_state, current_step)
        
        goal_handle.succeed()
        
        # Definieer en retourneer het resultaat
        result = PickAndPlace.Result()
        result.success = True
        # result.error_code = ...
        # result.error_message = ...
        return result

def main(args=None):
    rclpy.init(args=args)
    motion_node = Motion()
    
    try:
        rclpy.spin(motion_node)
    except KeyboardInterrupt:
        pass
    finally:
        motion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()