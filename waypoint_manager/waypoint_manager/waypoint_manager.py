import rclpy
from rclpy.node import Node


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        self.get_logger().info('Initialized')


def main():
    rclpy.init()
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()