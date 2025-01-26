# src/fg_srv/fg_srv/rn1.py

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from fg_srv.action import Mission
import requests
import json

class MissionRetrievalActionClient(Node):
    def __init__(self):
        super().__init__('mission_retrieval_action_client')
        
        # Action Client
        self._action_client = ActionClient(self, Mission, 'execute_mission')
        
        # REST API endpoint
        self.mission_api_url = "http://localhost:8000/mission"
        
        # Timer for periodic mission retrieval and action calls
        self.create_timer(1.0, self.retrieve_and_send_mission)

    def retrieve_and_send_mission(self):
        """
        Retrieve mission data from REST API and send as ROS2 action.
        """
        try:
            response = requests.get(self.mission_api_url)
            response.raise_for_status()
            mission_data = response.json()
            
            self.get_logger().info(f"Mission retrieved: {mission_data}")
            
            # Send as action
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Action server not available")
                return

            goal_msg = Mission.Goal()
            goal_msg.mission_id = mission_data['mission_id']
            goal_msg.mission_type = mission_data['mission_type']
            goal_msg.latitude = mission_data['target_coordinates']['latitude']
            goal_msg.longitude = mission_data['target_coordinates']['longitude']
            goal_msg.priority = mission_data['priority']

            self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            ).add_done_callback(self.goal_response_callback)
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error retrieving mission: {e}")

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the Action Server.
        """
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback.progress:.2f}% completed")

    def goal_response_callback(self, future):
        """
        Handle response to the goal sent to the Action Server.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Action Server")
            return

        self.get_logger().info("Goal accepted by Action Server")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Handle the result from the Action Server.
        """
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Mission execution completed successfully: {result.message}")
        else:
            self.get_logger().error(f"Mission execution failed: {result.message}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionRetrievalActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()