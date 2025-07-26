import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy)
import math

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class RacetrackController(Node):
    def __init__(self):
        super().__init__('racetrack_controller')

        # Parameters
        self.dry_run = False

        self.lookahead_distance = 1.5
        self.max_linear_speed = 2.5
        self.max_angular_speed = 2.0

        self.odom_link_name = 'fastbot_1_odom'

        self.last_odom_time = None
        self.last_position = None
        self.last_yaw = None

        # Subscribers
        transient_local_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.create_subscription(Path, '/waypoint/plan', self.path_callback, transient_local_qos)
        self.create_subscription(Odometry, '/fastbot_1/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/fastbot_1/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/fastbot_1/local_plan', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/fastbot_1/target_marker', 10)

        self.path = []
        self.received_path = False

    def publish_zero_velocity(self):
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.received_path = True
        self.get_logger().info(f'Received path with {len(self.path)} poses.')

    def odom_callback(self, msg: Odometry):
        if not self.received_path or not self.path:
            return

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        yaw = self.normalize_angle(self.get_yaw_from_quaternion(msg.pose.pose.orientation) - math.pi)

        if self.last_position is None or self.last_odom_time is None:
            self.last_position = position
            self.last_yaw = yaw
            self.last_odom_time = current_time
            return

        dt = current_time - self.last_odom_time
        if dt <= 0:
            return

        # Estimate actual velocities from pose delta
        dx = position.x - self.last_position.x
        dy = position.y - self.last_position.y
        dyaw = self.normalize_angle(yaw - self.last_yaw)

        actual_linear_vel = math.sqrt(dx**2 + dy**2) / dt
        actual_angular_vel = dyaw / dt

        # Store for next cycle
        self.last_position = position
        self.last_yaw = yaw
        self.last_odom_time = current_time

        target = self.find_lookahead_point(position)
        if target is None:
            self.get_logger().warn('No valid lookahead point found.')
            if not self.dry_run:
                self.publish_zero_velocity()
            return

        self.publish_target_marker(target)

        # Compute control command
        dx = target.pose.position.x - position.x
        dy = target.pose.position.y - position.y
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - yaw)

        cmd = Twist()
        cmd.linear.x = self.max_linear_speed
        cmd.angular.z = max(-self.max_angular_speed,
                            min(self.max_angular_speed, angle_diff))

        # Print command vs actual
        self.get_logger().info(
            f"[CMD] linear: {cmd.linear.x:.3f}, angular: {cmd.angular.z:.3f} | "
            f"[ACTUAL] linear: {actual_linear_vel:.3f}, angular: {actual_angular_vel:.3f}"
        )

        if not self.dry_run:
            self.cmd_pub.publish(cmd)

        self.publish_local_plan(position, yaw, cmd.linear.x, cmd.angular.z)

    def find_lookahead_point(self, current_position):
        if not self.path:
            return None

        # Find the closest point on the path to the current position first
        closest_index = None
        min_dist = float('inf')

        for i, pose_stamped in enumerate(self.path):
            pos = pose_stamped.pose.position
            dx = pos.x - current_position.x
            dy = pos.y - current_position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        # Starting from closest_index, move forward along the path until
        # we find a point at least lookahead_distance away
        path_length = len(self.path)
        total_dist = 0.0
        last_pos = self.path[closest_index].pose.position

        for i in range(1, path_length + 1):  # +1 to allow wrap-around search
            index = (closest_index + i) % path_length
            pos = self.path[index].pose.position
            dx = pos.x - last_pos.x
            dy = pos.y - last_pos.y
            segment_dist = math.sqrt(dx**2 + dy**2)
            total_dist += segment_dist
            last_pos = pos

            if total_dist >= self.lookahead_distance:
                return self.path[index]

        # If we didn't find a point far enough ahead, return the closest point
        return self.path[closest_index]
       
    def publish_local_plan(self, current_position, yaw, linear_vel, angular_vel):
        local_plan = Path()
        local_plan.header.frame_id = self.odom_link_name
        local_plan.header.stamp = self.get_clock().now().to_msg()

        # Simulation parameters
        dt = 0.1  # time step [s]
        num_steps = 10  # plan 1 second ahead

        x, y, theta = current_position.x, current_position.y, yaw

        for _ in range(num_steps):
            # Update pose using simple bicycle/kinematic model
            x += linear_vel * math.cos(theta) * dt
            y += linear_vel * math.sin(theta) * dt
            theta += angular_vel * dt

            pose = PoseStamped()
            pose.header = local_plan.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
            local_plan.poses.append(pose)

        self.local_plan_pub.publish(local_plan)

    def publish_target_marker(self, target_pose):
        marker = Marker()
        marker.header.frame_id = self.odom_link_name
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lookahead_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = target_pose.pose

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.target_marker_pub.publish(marker)

    @staticmethod
    def get_yaw_from_quaternion(q):
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = RacetrackController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C pressed. Stopping robot...')
        try:
            node.publish_zero_velocity()
        except Exception as e:
            node.get_logger().error(f'Failed to publish stop command: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
