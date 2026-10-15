#!/usr/bin/env python3
# this is used as a baseline for sih where if paths intersect lower priority robot stops and waits for higher priority robot to pass.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from turtlebot3_multi_robot.msg import RobotState
import math
import numpy as np

ROBOTS = ['tb1', 'tb2', 'tb3', 'tb4']
ROBOT_RADIUS = 0.2
COLLISION_DIST = ROBOT_RADIUS * 2.2 # 0.44m

class BaselineP2PNode(Node):
    def __init__(self):
        super().__init__('baseline_p2p_node')
        
        raw_ns = self.get_namespace().strip('/')
        self.robot_id = raw_ns if raw_ns else 'tb1'
        
        self.peers = [r for r in ROBOTS if r != self.robot_id]
        
        self.current_pose = None
        self.current_vel = Twist()
        self.current_goal = None
        self.current_path = [] # list of Points
        self.peer_states = {}
        
        # Subscribers for self state
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.create_subscription(Path, 'plan', self.plan_callback, 10)
        
        # Intercept cmd_vel_nav published by nav2 controller,  and output to true cmd_vel
        self.create_subscription(Twist, 'cmd_vel_nav', self.cmd_vel_nav_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # P2P Network
        self.state_pub = self.create_publisher(RobotState, '/p2p_network', 10)
        self.create_subscription(RobotState, '/p2p_network', self.p2p_callback, 10)
        
        self.timer = self.create_timer(0.1, self.broadcast_state)
        
        self.get_logger().info(f"Baseline Stop-and-Yield Node started for {self.robot_id}")

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose
        self.current_vel = msg.twist.twist
        
    def goal_callback(self, msg):
        self.current_goal = msg
        
    def plan_callback(self, msg):
        # Extract the next 2.0 meters of path
        preview = []
        if self.current_pose:
            # setting min distance to infinity  
            min_dist = float('inf')
            closest_idx = 0
            # msg.poses contains the path from start to goal , from which we filter out current 2 m 
            # While the path sequence is ordered sequentially from start to finish,
            #  index 0 is only the closest point(the point closest to the bot) at the exact moment the robot begins its journey.
            for i, p in enumerate(msg.poses):
                dist = math.hypot(p.pose.position.x - self.current_pose.pose.position.x,
                                  p.pose.position.y - self.current_pose.pose.position.y)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
                    
           
            accumulated_dist = 0.0
            
            last_p = msg.poses[closest_idx].pose.position
            preview.append(last_p)

            # iterates all variables after closest_idx to get relevant part of the path. 
            for i in range(closest_idx + 1, len(msg.poses)):
                p = msg.poses[i].pose.position
                dist = math.hypot(p.x - last_p.x, p.y - last_p.y)
                accumulated_dist += dist
                
                # Sample roughly every 0.1m to keep msg size small
                if dist >= 0.05 or len(preview) == 1: 
                    preview.append(p)
                    last_p = p
                    
                if accumulated_dist > 2.0:
                    break
        else:
            # Just take first 20 points if no odom yet
            preview = [p.pose.position for p in msg.poses[:20]]
            
        self.current_path = preview
   
    def p2p_callback(self, msg):
        if msg.robot_id != self.robot_id:
            self.peer_states[msg.robot_id] = msg
            
    def broadcast_state(self):
        if not self.current_pose:
            return
            
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.pose = self.current_pose
        msg.velocity = self.current_vel
        msg.stamp = self.get_clock().now().to_msg()
        msg.path_preview = self.current_path
        
        if self.current_goal:
            msg.goal = self.current_goal
            
        self.state_pub.publish(msg)
        
    def check_intersection(self):
        # Returns True if we need to yield ( stop and wait for higher priority robot to pass)
        for peer_id, peer_state in self.peer_states.items():
            # Check if peer is higher priority (lower string value, so tb1 > tb2)
            if peer_id < self.robot_id:
                # Check path intersection
                if not self.current_path or not peer_state.path_preview:
                    continue
                    
                for my_p in self.current_path:
                    for their_p in peer_state.path_preview:
                        dist = math.hypot(my_p.x - their_p.x, my_p.y - their_p.y)
                        if dist < COLLISION_DIST:
                            # Our paths cross within 2 meters! 
                            # Since they are higher priority, we MUST yield.
                            return True
        return False

    def cmd_vel_nav_callback(self, msg):
        if not self.current_pose:
            self.cmd_vel_pub.publish(msg)
            return
            
        should_yield = self.check_intersection()
        
        if should_yield:
            # Stop and Yield Baseline!
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
            self.get_logger().info("Yielding to higher priority robot!", throttle_duration_sec=1.0)
        else:
            # Pass through exactly what Nav2 wants
            self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BaselineP2PNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
