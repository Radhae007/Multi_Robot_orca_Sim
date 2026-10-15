#!/usr/bin/env python3
"""
Fully commented ORCA Algorithm codes sept 7 21:50 - radhae 
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener, TransformException
from turtlebot3_multi_robot.msg import RobotState


class OrcaP2PNode(Node):
    def __init__(self):
        super().__init__('orca_p2p_node')

        # Declare ROS 2 parameters
        self.declare_parameter('d_threshold', 8.0)
        self.declare_parameter('broadcast_rate', 10.0)
        self.declare_parameter('peer_timeout', 2.0)
        self.declare_parameter('global_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Do not enforce type for enable_orca so launch strings don't crash it
        self.declare_parameter('enable_orca', rclpy.Parameter.Type.PARAMETER_NOT_SET)

        self.d_threshold = float(self.get_parameter('d_threshold').value)
        self.broadcast_rate = float(self.get_parameter('broadcast_rate').value)
        self.peer_timeout = float(self.get_parameter('peer_timeout').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        
        orca_val = self.get_parameter('enable_orca').value
        if isinstance(orca_val, str):
            self.enable_orca = (orca_val.lower() == 'true')
        elif orca_val is None:
            self.enable_orca = True
        else:
            self.enable_orca = bool(orca_val)

        # In below ns the / is removed , only then is the addition of a prefix to the namespace is considered. 
        # Otherwise, the namespace is considered absolute and prefix will get ignored. This is a ROS2 launch nuance.
        raw_ns = self.get_namespace().strip('/')
        self.robot_id = raw_ns if raw_ns else 'tb_host'
        self.get_logger().info(f"Initializing ORCA P2P Node for Robot ID: '{self.robot_id}'")

        # TF Buffer & Listener for pose extraction
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Local state storage
        self.current_goal = PoseStamped()
        self.current_velocity = Twist()
        self.peer_registry = {}  # {robot_id: {'pose': PoseStamped, 'velocity': Twist, 'stamp': float}}

        # QoS configuration for low-latency DDS state sharing
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        # Global P2P shared topic (using root namespace '/p2p/robot_states')
        self.p2p_pub = self.create_publisher(RobotState, '/p2p/robot_states', qos)
        # Namespace-local proximity flag
        self.orca_zone_pub = self.create_publisher(Bool, 'in_orca_zone', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.p2p_sub = self.create_subscription(
            RobotState, '/p2p/robot_states', self.p2p_callback, qos
        )
        self.cmd_vel_nav_sub = self.create_subscription(
            Twist, 'cmd_vel_nav', self.cmd_vel_nav_callback, 10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for state broadcasting and distance computation
        timer_period = 1.0 / self.broadcast_rate
        self.timer = self.create_timer(timer_period, self.timer_loop)
        
        self.latest_cmd_vel_nav = Twist()
        self.in_zone = False

    def cmd_vel_nav_callback(self, msg: Twist):
        self.latest_cmd_vel_nav = msg

    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg

    def odom_callback(self, msg: Odometry):
        self.current_velocity = msg.twist.twist

    def p2p_callback(self, msg: RobotState):
        # Skip self-messages
        if msg.robot_id == self.robot_id:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.peer_registry[msg.robot_id] = {
            'pose': msg.pose,
            'goal': msg.goal,
            'velocity': msg.velocity,
            'last_seen': now_sec
        }

    def get_current_host_pose(self):
        # here host is the robot running this code 
        try:
            # In Nav2 multi-robot, global frame is usually 'map' (no namespace).
            # But if there's no map server, it's 'odom', which DOES have a namespace!

            # value_if_true if condition else value_if_false -- syntax of below line 
            global_frame_ns = f"{self.robot_id}/{self.global_frame}" if (self.robot_id and self.robot_id != 'tb_host' and self.global_frame != 'map') else self.global_frame
            
            # Base frame has the namespace, e.g. 'tb1/base_footprint'
            base_frame_ns = f"{self.robot_id}/{self.base_frame}" if self.robot_id and self.robot_id != 'tb_host' else self.base_frame
            
            tf_stamped = self.tf_buffer.lookup_transform(
                global_frame_ns,
                base_frame_ns,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05)
            )
            pose = PoseStamped()
            pose.header = tf_stamped.header
            pose.pose.position.x = tf_stamped.transform.translation.x
            pose.pose.position.y = tf_stamped.transform.translation.y
            pose.pose.position.z = tf_stamped.transform.translation.z
            pose.pose.orientation = tf_stamped.transform.rotation
            return pose
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed from {global_frame_ns} to {base_frame_ns}: {ex}")
            return None

    def get_yaw(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def compute_rvo_velocity(self, host_pose, peers):
        # Desired velocity from Nav2
        v_des = self.latest_cmd_vel_nav.linear.x
        w_des = self.latest_cmd_vel_nav.angular.z
        
        # Current yaw
        yaw = self.get_yaw(host_pose.pose.orientation)
        
        hx = host_pose.pose.position.x
        hy = host_pose.pose.position.y
        
        # Sample space (Dynamic Window around Nav2's target velocity)
        # v: 0.0 to 0.22 (max tb3 speed)
        # w: -2.84 to 2.84 (max tb3 angular)
        best_v = 0.0
        best_w = 0.0
        min_penalty = float('inf')
        
        robot_radius = 0.4
        
        # Adaptive Control Sampling: finely sample around the desired velocity
        v_samples = set([max(0.0, min(0.22, v_des + dv)) for dv in [-0.1, -0.05, 0.0, 0.05, 0.1]])
        w_samples = set([max(-2.84, min(2.84, w_des + dw)) for dw in [-1.0, -0.5, 0.0, 0.5, 1.0]])
        
        for v in v_samples:
            for w in w_samples:
                # Penalty 1: difference from desired velocity
                penalty = abs(v - v_des) + 0.5 * abs(w - w_des)
                
                # Reciprocal Right-Hand Passing Bias
                # Turning right (w < 0) reduces the penalty slightly, breaking head-on symmetry
                right_hand_bias = w * 0.1
                penalty += right_hand_bias
                
                # Predict future position in 1.0 second
                pred_x = hx + v * math.cos(yaw + w)
                pred_y = hy + v * math.sin(yaw + w)
                
                collision_penalty = 0.0
                
                for peer in peers:
                    px = peer['pose'].pose.position.x
                    py = peer['pose'].pose.position.y
                    
                    p_yaw = self.get_yaw(peer['pose'].pose.orientation)
                    p_v = peer['velocity'].linear.x
                    p_w = peer['velocity'].angular.z
                    
                    peer_pred_x = px + p_v * math.cos(p_yaw + p_w)
                    peer_pred_y = py + p_v * math.sin(p_yaw + p_w)
                    
                    dist = math.hypot(pred_x - peer_pred_x, pred_y - peer_pred_y)
                    
                    if dist < self.d_threshold:
                        # Smooth Exponential Penalty Function
                        # Creates a continuous cost gradient instead of a harsh inverse spike
                        collision_penalty += 500.0 * math.exp(-3.0 * dist)
                
                total_penalty = penalty + collision_penalty
                if total_penalty < min_penalty:
                    min_penalty = total_penalty
                    best_v = v
                    best_w = w
                    
        return best_v, best_w

    def timer_loop(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        host_pose = self.get_current_host_pose()

        if host_pose is None:
            return

        # 1. Broadcast Host State
        state_msg = RobotState()
        state_msg.robot_id = self.robot_id
        state_msg.pose = host_pose
        state_msg.goal = self.current_goal
        state_msg.velocity = self.current_velocity
        state_msg.stamp = self.get_clock().now().to_msg()
        self.p2p_pub.publish(state_msg)

        # 2. Prune Stale Peers & Evaluate Distances
        self.in_zone = False
        expired_keys = []
        valid_peers = []

        hx = host_pose.pose.position.x
        hy = host_pose.pose.position.y

        for peer_id, peer_data in self.peer_registry.items():
            if now_sec - peer_data['last_seen'] > self.peer_timeout:
                expired_keys.append(peer_id)
                continue

            valid_peers.append(peer_data)
            px = peer_data['pose'].pose.position.x
            py = peer_data['pose'].pose.position.y
            dist = math.hypot(hx - px, hy - py)

            if dist <= self.d_threshold:
                self.in_zone = True

        for k in expired_keys:
            del self.peer_registry[k]

        # 3. Publish Proximity Flag
        zone_msg = Bool()
        zone_msg.data = self.in_zone
        self.orca_zone_pub.publish(zone_msg)
        
        # 4. Velocity Override Logic
        out_twist = Twist()
        if not self.enable_orca or not self.in_zone:
            # Safe zone (or ORCA disabled): Follow Nav2 blindly
            out_twist = self.latest_cmd_vel_nav
        else:
            # Danger zone & ORCA enabled: Apply custom RVO sampling algorithm
            v, w = self.compute_rvo_velocity(host_pose, valid_peers)
            out_twist.linear.x = v
            out_twist.angular.z = w
            
        self.cmd_vel_pub.publish(out_twist)


def main(args=None):
    rclpy.init(args=args)
    node = OrcaP2PNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
