# src/fg_srv/fg_srv/rn2.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from fg_srv.action import Mission

class MissionExecutionActionServer(Node):
    def __init__(self):
        super().__init__('mission_execution_action_server')
        
        # Action Server
        self._action_server = ActionServer(
            self,
            Mission,
            'execute_mission',
            self.execute_callback
        )
        self.get_logger().info("Mission Execution Action Server is ready")

    def execute_callback(self, goal_handle):
        """
        Handle the execution of a received action goal.
        """
        self.get_logger().info(f"Received mission: {goal_handle.request.mission_id}")
        
        try:
            # Simulate execution progress
            for progress in range(0, 101, 10):
                self.get_logger().info(f"Executing mission... {progress}% complete")
                goal_handle.publish_feedback(Mission.Feedback(progress=progress))
                self.get_clock().sleep_for(1.0)

            goal_handle.succeed()
            result = Mission.Result()
            result.success = True
            result.message = f"Mission {goal_handle.request.mission_id} executed successfully"
            return result

        except Exception as e:
            goal_handle.abort()
            result = Mission.Result()
            result.success = False
            result.message = f"Mission execution failed: {str(e)}"
            return result

def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutionActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()