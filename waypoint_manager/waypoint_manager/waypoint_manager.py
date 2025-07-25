import rclpy
from rclpy.node import Node

from waypoint_msgs.msg import *
from waypoint_msgs.srv import *


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.append_srv = self.create_service(AppendWaypoint, 'waypoint/append', self.append_cb)
        self.delete_srv = self.create_service(DeleteWaypoint, 'waypoint/delete', self.delete_cb)
        self.insert_srv = self.create_service(InsertWaypoint, 'waypoint/insert', self.insert_cb)
        self.load_srv = self.create_service(LoadWaypoints, 'waypoint/load', self.load_cb)
        self.overwrite_srv = self.create_service(OverwriteWaypoint, 'waypoint/overwrite', self.overwrite_cb)
        self.save_srv = self.create_service(SaveWaypoints, 'waypoint/save', self.save_cb)
        
        self.get_logger().info('Initialized')

    def append_cb(self, request, response):
        self.get_logger().info('append called')

        response.success = False
        response.message = 'Missing implementation'
        return response

    def delete_cb(self, request, response):
        self.get_logger().info('delete called')

        response.success = False
        response.message = 'Missing implementation'
        return response

    def insert_cb(self, request, response):
        self.get_logger().info('insert called')

        response.success = False
        response.message = 'Missing implementation'
        return response

    def load_cb(self, request, response):
        self.get_logger().info('load called')

        response.success = False
        response.message = 'Missing implementation'
        return response

    def overwrite_cb(self, request, response):
        self.get_logger().info('overwrite called')

        response.success = False
        response.message = 'Missing implementation'
        return response

    def save_cb(self, request, response):
        self.get_logger().info('save called')

        response.success = False
        response.message = 'Missing implementation'
        return response


def main():
    rclpy.init()
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()