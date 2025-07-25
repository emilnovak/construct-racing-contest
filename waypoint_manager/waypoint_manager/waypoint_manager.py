import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException

from waypoint_msgs.msg import *
from waypoint_msgs.srv import *


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.map_link_name = 'fastbot_1_odom'
        self.base_link_name = 'fastbot_1_base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # initialize waypoints
        self.waypoints = []

        self.append_srv = self.create_service(AppendWaypoint, 'waypoint/append', self.append_cb)
        self.delete_srv = self.create_service(DeleteWaypoint, 'waypoint/delete', self.delete_cb)
        self.insert_srv = self.create_service(InsertWaypoint, 'waypoint/insert', self.insert_cb)
        self.load_srv = self.create_service(LoadWaypoints, 'waypoint/load', self.load_cb)
        self.overwrite_srv = self.create_service(OverwriteWaypoint, 'waypoint/overwrite', self.overwrite_cb)
        self.save_srv = self.create_service(SaveWaypoints, 'waypoint/save', self.save_cb)
        
        self.get_logger().info('Initialized')

    def append_cb(self, request, response):
        self.get_logger().info('append called')

        try:
            pose = self.tf_buffer.lookup_transform(self.map_link_name, self.base_link_name, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException as e:
            response.success = False
            response.message = f'Tranform exception: {e}'
            return response

        self.waypoints.append(pose.transform)
        response.success = True
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