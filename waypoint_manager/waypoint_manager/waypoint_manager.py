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
from copy import deepcopy
from yaml import dump, safe_load

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.map_link_name = 'fastbot_1_odom'
        self.base_link_name = 'fastbot_1_base_link'

        self.flatten_transforms = True

        self.map_path = '/home/user/ros2_ws/src/construct-racing-contest/map/waypoints.yaml'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []

        self.marker_publisher = self.create_publisher(Marker, 'waypoints', 10)

        self.append_srv = self.create_service(AppendWaypoint, 'waypoint/append', self.append_cb)
        self.delete_srv = self.create_service(DeleteWaypoint, 'waypoint/delete', self.delete_cb)
        self.insert_srv = self.create_service(InsertWaypoint, 'waypoint/insert', self.insert_cb)
        self.load_srv = self.create_service(LoadWaypoints, 'waypoint/load', self.load_cb)
        self.overwrite_srv = self.create_service(OverwriteWaypoint, 'waypoint/overwrite', self.overwrite_cb)
        self.save_srv = self.create_service(SaveWaypoints, 'waypoint/save', self.save_cb)
        
        self.get_logger().info('Initialized')

        # Load markers after some spin
        self.load_map_timer = self.create_timer(
            1.0,
            lambda: (
                self.get_logger().info('Load map lambda runs'),
                self.load_map_timer.cancel(),
                self.load_map()
            )
        )


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
        waypoints_data = [{
            'position': {
                'x': float(p.position.x),
                'y': float(p.position.y),
                'z': float(p.position.z),
            },
            'orientation': {
                'x': float(p.orientation.x),
                'y': float(p.orientation.y),
                'z': float(p.orientation.z),
                'w': float(p.orientation.w),
            }
        } for p in self.waypoints]

        try:
            with open(self.map_path, 'w') as f:
                dump(waypoints_data, f)
            response.success = True
        except Exception as e:
            self.get_logger().error('Error saving map yaml: {e}')
            response.success = False
            response.message = str(e)

        return response

    def load_map(self):
        try:
            with open(self.map_path, 'r') as f:
                data = safe_load(f)

            self.waypoints = []
            for item in data:
                pose = Pose()
                pose.position.x = item['position']['x']
                pose.position.y = item['position']['y']
                pose.position.z = item['position']['z']
                pose.orientation.x = item['orientation']['x']
                pose.orientation.y = item['orientation']['y']
                pose.orientation.z = item['orientation']['z']
                pose.orientation.w = item['orientation']['w']
                self.waypoints.append(pose)

        except Exception as e:
            self.get_logger().error(f'Error while loading map yaml: {e}')

        self.publish_markers()

    def publish_markers(self):
        self.get_logger().info('publishing markers')

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = self.map_link_name
        self.marker_publisher.publish(delete_marker)

        # LINESTRIP
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = self.map_link_name
        line_strip_marker.ns = 'waypoint_lines'
        line_strip_marker.id = 9999
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.1
        line_strip_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
        line_strip_marker.points = []

        for i, waypoint in enumerate(self.waypoints):

            # Common marker properties
            base_marker = Marker()

            base_marker.header.frame_id = self.map_link_name
            base_marker.action = Marker.ADD
            base_marker.pose.position = waypoint.position
            base_marker.pose.orientation = waypoint.orientation

            # ARROW markers
            arrow_marker = deepcopy(base_marker)

            arrow_marker.ns = 'waypoints'
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.scale.x = 1.0
            arrow_marker.scale.y = 0.1
            arrow_marker.scale.z = 0.1
            arrow_marker.color = ColorRGBA(r=1.0, g=.0, b=.0, a=0.5)

            self.marker_publisher.publish(arrow_marker)

            # TEXT markers
            text_marker = deepcopy(base_marker)
            text_marker.ns = 'waypoint_id'
            text_marker.id = i + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.pose.position.y = base_marker.pose.position.y + 0.1
            text_marker.pose.position.z = base_marker.pose.position.z + 0.2
            text_marker.scale.z = 0.5
            text_marker.text = str(i)

            self.marker_publisher.publish(text_marker)

            # LINE_STRIP markers
            line_strip_marker.points.append(waypoint.position)

        # Connect first and last waypoints and publish
        if self.waypoints and len(self.waypoints) > 1:
            line_strip_marker.points.append(self.waypoints[0].position)

            self.marker_publisher.publish(line_strip_marker)


def main():
    rclpy.init()
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()