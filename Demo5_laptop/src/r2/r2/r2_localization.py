import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped


class MapFrameConstants:
    DEFAULT_X_OFFSET = 0.8
    DEFAULT_Y_OFFSET = 1.8
    ARENA_X_OFFSET   = 11.2
    ARENA_Y_OFFSET   = 5.2
    ARENA_Z_OFFSET   = 0.4


class RobotPosePublisher(Node):

    def __init__(self):
        super().__init__('robot_pose_publisher')

        self.declare_parameter('starting_zone',          'Default')
        self.declare_parameter('use_world_map',          True)
        self.declare_parameter('robot_facing_spearhead', False)
        self.declare_parameter('playing_as',             'Red')

        self.starting_zone          = self.get_parameter('starting_zone').get_parameter_value().string_value
        self.use_world_map          = self.get_parameter('use_world_map').get_parameter_value().bool_value
        self.robot_facing_spearhead = self.get_parameter('robot_facing_spearhead').get_parameter_value().bool_value
        self.playing_as             = self.get_parameter('playing_as').get_parameter_value().string_value

        self.c = MapFrameConstants()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub     = self.create_publisher(PoseStamped, '/r2/pose',     10)
        self.map_pose_pub = self.create_publisher(PoseStamped, '/r2/map_pose', 10)
        self.create_timer(0.1, self.publish_pose)  # 10 Hz

    # ── Coordinate remapping ──────────────────────────────────────────────────

    def _apply_world_offset(self, raw: PoseStamped) -> PoseStamped:
        """Remap the raw GLIM pose into the competition world frame."""
        c   = self.c
        x   = raw.pose.position.x
        y   = raw.pose.position.y
        red = self.playing_as == 'Red'

        # Y offset flips sign for the opposing (Blue) side
        y_sign = 1.0 if red else -1.0

        if self.starting_zone == 'Default':
            x_off = c.DEFAULT_X_OFFSET
            y_off = c.DEFAULT_Y_OFFSET * y_sign
            z_off = 0.0
        else:  # Arena
            self.robot_facing_spearhead = True  # Arena starts rotated 90°
            x_off = c.DEFAULT_X_OFFSET + c.ARENA_X_OFFSET
            y_off = c.ARENA_Y_OFFSET   * y_sign
            z_off = c.ARENA_Z_OFFSET

        mapped = PoseStamped()
        mapped.header = raw.header
        mapped.pose.orientation     = raw.pose.orientation
        mapped.pose.position.z      = raw.pose.position.z + z_off

        if self.robot_facing_spearhead:
            # Robot faces spearhead (rotated 90°): GLIM lateral→world X, GLIM forward→world Y
            mapped.pose.position.x = -y + x_off
            mapped.pose.position.y = x - y_off
        else:
            mapped.pose.position.x = x + x_off
            mapped.pose.position.y = y - y_off

        return mapped

    # ── Main timer callback ───────────────────────────────────────────────────

    def publish_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'imu', rclpy.time.Time())
        except Exception:
            return

        raw = PoseStamped()
        raw.header.stamp     = self.get_clock().now().to_msg()
        raw.header.frame_id  = f'map_{self.playing_as.lower()}'
        raw.pose.position.x  = tf.transform.translation.x
        raw.pose.position.y  = tf.transform.translation.y
        raw.pose.position.z  = tf.transform.translation.z
        raw.pose.orientation = tf.transform.rotation

        self.pose_pub.publish(raw)

        if self.use_world_map:
            self.map_pose_pub.publish(self._apply_world_offset(raw))


def main():
    rclpy.init()
    rclpy.spin(RobotPosePublisher())


if __name__ == '__main__':
    main()
