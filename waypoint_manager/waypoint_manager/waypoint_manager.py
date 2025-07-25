import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from .utilities import euler_from_quat, quat_from_euler

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from waypoint_msgs.msg import *
from waypoint_msgs.srv import *

from math import pi

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.map_link_name = 'fastbot_1_odom'
        self.base_link_name = 'fastbot_1_base_link'

        self.flatten_transforms = True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # initialize waypoints
        self.waypoints = []

        self.marker_publisher = self.create_publisher(Marker, 'waypoints', 10)

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
            lookup = self.tf_buffer.lookup_transform(self.map_link_name, self.base_link_name, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException as e:
            response.success = False
            response.message = f'Tranform exception: {e}'
            return response

        pose = Pose()
        pose.position.x = lookup.transform.translation.x
        pose.position.y = lookup.transform.translation.y
        pose.position.z = lookup.transform.translation.z
        pose.orientation.x = lookup.transform.rotation.x
        pose.orientation.y = lookup.transform.rotation.y
        pose.orientation.z = lookup.transform.rotation.z
        pose.orientation.w = lookup.transform.rotation.w

        # project to xy plane
        if self.flatten_transforms:
            pose.position.z = .0

            (roll, pitch, yaw) = euler_from_quat(
                lookup.transform.rotation.x,
                lookup.transform.rotation.y,
                lookup.transform.rotation.z,
                lookup.transform.rotation.w,
            )

            x, y, z, w = quat_from_euler(0, 0, yaw+pi)

            pose.orientation.x = x
            pose.orientation.y = y
            pose.orientation.z = z
            pose.orientation.w = w

        self.waypoints.append(pose)
        self.publish_markers()

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

    def publish_markers(self):

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = self.map_link_name
        self.marker_publisher.publish(delete_marker)

        for i, waypoint in enumerate(self.waypoints):
            m = Marker()

            m.header.frame_id = self.map_link_name
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD

            m.pose.position = waypoint.position
            m.pose.orientation = waypoint.orientation

            m.scale.x = 0.5
            m.scale.y = 0.1
            m.scale.z = 0.1

            m.color = ColorRGBA(r=1.0, g=.0, b=.0, a=1.0)

            self.marker_publisher.publish(m)


def main():
    rclpy.init()
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()