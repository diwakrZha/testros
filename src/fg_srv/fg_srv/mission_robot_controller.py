#src/fg_srv/fg_srv/mission_robot_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import geometry_msgs.msg
import math

class MissionRobotController(Node):
    def __init__(self):
        super().__init__('mission_robot_controller')
        
        # Subscribe to mission topic
        self.subscription = self.create_subscription(
            String,
            'mission_topic',
            self.mission_callback,
            10
        )
        
        # Publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(
            geometry_msgs.msg.Twist,
            '/mission_robot/cmd_vel',
            10
        )
        
        # Current target coordinates
        self.target_lat = None
        self.target_lon = None
        
        # Current robot pose (placeholder for actual pose tracking)
        self.current_lat = 0.0
        self.current_lon = 0.0
        
        # Timer for continuous navigation
        self.navigation_timer = self.create_timer(0.5, self.navigate_to_coordinates)
        
        self.get_logger().info('Mission Robot Controller initialized')

    def mission_callback(self, msg):
        try:
            # Parse mission JSON
            mission = json.loads(msg.data)
            
            # Extract target coordinates
            target_coords = mission.get('target_coordinates', {})
            self.target_lat = target_coords.get('latitude')
            self.target_lon = target_coords.get('longitude')
            
            if self.target_lat is not None and self.target_lon is not None:
                self.get_logger().info(f'Received mission to coordinates: {self.target_lat}, {self.target_lon}')
        
        except json.JSONDecodeError:
            self.get_logger().error('Invalid mission JSON')
        except Exception as e:
            self.get_logger().error(f'Error processing mission: {e}')

    def navigate_to_coordinates(self):
        if self.target_lat is None or self.target_lon is None:
            return
        
        # Calculate distance and bearing
        distance = self.haversine_distance(
            self.current_lat, self.current_lon, 
            self.target_lat, self.target_lon
        )
        
        # Stopping threshold
        if distance < 0.001:  # About 1 meter
            self.get_logger().info('Reached target coordinates')
            self.target_lat = None
            self.target_lon = None
            return
        
        # Calculate heading
        bearing = self.calculate_bearing(
            self.current_lat, self.current_lon, 
            self.target_lat, self.target_lon
        )
        
        # Create movement command
        move_cmd = geometry_msgs.msg.Twist()
        move_cmd.linear.x = min(distance * 10, 1.0)  # Speed proportional to distance
        move_cmd.angular.z = bearing  # Rotate to align with target
        
        self.get_logger().info(f'Navigating: Distance={distance}, Bearing={bearing}, Speed={move_cmd.linear.x}')
        self.cmd_vel_pub.publish(move_cmd)
        
        # Update current position (simulated - in real world, this would come from localization)
        self.current_lat += distance * math.cos(bearing) * 0.0001
        self.current_lon += distance * math.sin(bearing) * 0.0001

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        # Haversine formula to calculate distance between two lat/lon points
        R = 6371  # Earth's radius in kilometers
        
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat/2) * math.sin(dlat/2) + 
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * 
             math.sin(dlon/2) * math.sin(dlon/2))
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        # Calculate initial bearing between two points
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        dlon = lon2 - lon1
        
        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) - 
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        
        initial_bearing = math.atan2(x, y)
        return initial_bearing

def main(args=None):
    rclpy.init(args=args)
    mission_controller = MissionRobotController()
    rclpy.spin(mission_controller)
    mission_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()