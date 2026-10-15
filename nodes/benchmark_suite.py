#!/usr/bin/env python3
# this node is used as 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import math
import time
import csv
import argparse
import random

ROBOTS = ['tb1', 'tb2', 'tb3', 'tb4']
# Using the coordinates provided by the user
SPAWNS = {
    'tb1': (-2.0, -1.5),
    'tb2': (-2.0, 1.5),
    'tb3': (2.0, -1.5),
    'tb4': (2.0, 1.5)
}

ROBOT_RADIUS = 0.5
COLLISION_THRESHOLD = ROBOT_RADIUS * 2.0
COLLISION_COOLDOWN = 2.0

class BenchmarkNode(Node):
    def __init__(self, trials, mode):
        super().__init__('benchmark_suite')
        self.trials = trials
        self.mode = mode
        
        # Set a fixed random seed so trials are 100% reproducible between runs!
        random.seed(42)
        
        self.nav_clients = {}
        self.odoms = {}
        self.last_positions = {}
        self.distances = {r: 0.0 for r in ROBOTS}
        
        self.poses = {r: None for r in ROBOTS}
        self.collision_count = 0
        self.last_collision_time = 0.0
        
        for r in ROBOTS:
            self.nav_clients[r] = ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose')
            self.create_subscription(Odometry, f'/{r}/odom', lambda msg, robot=r: self.odom_callback(msg, robot), 10)
            
        self.timer = self.create_timer(0.1, self.collision_check_loop)
        
        # Results storage
        self.results = []
        
    def odom_callback(self, msg, robot):
        pos = msg.pose.pose.position
        self.poses[robot] = pos
        
        if robot in self.last_positions:
            last = self.last_positions[robot]
            dist = math.hypot(pos.x - last.x, pos.y - last.y)
            self.distances[robot] += dist
            
        self.last_positions[robot] = pos

    def collision_check_loop(self):
        # Check all pairs
        now = time.time()
        if now - self.last_collision_time < COLLISION_COOLDOWN:
            return
            
        for i in range(len(ROBOTS)):
            for j in range(i + 1, len(ROBOTS)):
                r1 = ROBOTS[i]
                r2 = ROBOTS[j]
                
                p1 = self.poses[r1]
                p2 = self.poses[r2]
                
                if p1 and p2:
                    dist = math.hypot(p1.x - p2.x, p1.y - p2.y)
                    if dist < COLLISION_THRESHOLD:
                        self.collision_count += 1
                        self.last_collision_time = now
                        self.get_logger().warn(f"COLLISION DETECTED between {r1} and {r2}! Distance: {dist:.2f}m")

    def run_benchmark(self):
        self.get_logger().info("Waiting for action servers...")
        for r, client in self.nav_clients.items():
            client.wait_for_server()
            
        self.get_logger().info("Action servers ready! Starting trials...")
        
        for trial in range(1, self.trials + 1):
            self.get_logger().info(f"--- STARTING TRIAL {trial}/{self.trials} ---")
            
            # Reset distances
            self.distances = {r: 0.0 for r in ROBOTS}
            self.collision_count = 0
            
            # Generate random targets for this trial (reproducible due to fixed seed)
            trial_targets = {}
            for r in ROBOTS:
                valid_target = False
                while not valid_target:
                    tx = random.uniform(-2.5, 2.5)
                    ty = random.uniform(-1.5, 1.5)
                    
                    # Check distance against already assigned targets
                    valid_target = True
                    for other_r, (ox, oy) in trial_targets.items():
                        if math.hypot(tx - ox, ty - oy) < 1.5:  # Minimum distance between targets
                            valid_target = False
                            break
                            
                trial_targets[r] = (tx, ty)
            
            # We use two dicts to distinguish goal futures and result futures
            goal_futures = {}
            result_futures = {}
            start_time = time.time()
            
            for r in ROBOTS:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                
                target_x, target_y = trial_targets[r]
                
                goal_msg.pose.pose.position.x = target_x
                goal_msg.pose.pose.position.y = target_y
                goal_msg.pose.pose.orientation.w = 1.0
                
                goal_futures[r] = self.nav_clients[r].send_goal_async(goal_msg)
                
            # Wait for all to finish
            completed = set()
            while len(completed) < len(ROBOTS):
                rclpy.spin_once(self, timeout_sec=0.1)
                
                for r in ROBOTS:
                    if r in completed:
                        continue
                        
                    if r in goal_futures and goal_futures[r].done():
                        res = goal_futures[r].result()
                        del goal_futures[r]
                        if not res.accepted:
                            self.get_logger().error(f"Goal rejected for {r}!")
                            completed.add(r)
                            continue
                        
                        result_futures[r] = res.get_result_async()
                        
                    elif r in result_futures and result_futures[r].done():
                        completed.add(r)
                        self.get_logger().info(f"{r} finished!")
                        
            end_time = time.time()
            duration = end_time - start_time
            
            total_dist = sum(self.distances.values())
            
            self.get_logger().info(f"TRIAL {trial} COMPLETE: Duration: {duration:.2f}s, Total Dist: {total_dist:.2f}m, Collisions: {self.collision_count}")
            
            self.results.append({
                'trial': trial,
                'mode': self.mode,
                'duration': round(duration, 2),
                'total_distance': round(sum(self.distances.values()), 2),
                'collisions': self.collision_count
            })
            
            # Wait a moment before next trial
            time.sleep(2.0)
            
        # Export
        filename = f"benchmark_results_{self.mode}.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['trial', 'mode', 'duration', 'total_distance', 'collisions'])
            writer.writeheader()
            writer.writerows(self.results)
        self.get_logger().info(f"Results saved to {filename}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=10, help='Number of trials to run')
    parser.add_argument('--mode', type=str, default='baseline', help='Mode to run: baseline, orca, or traffic')
    
    # We strip out ROS arguments
    import sys
    args, unknown = parser.parse_known_args(sys.argv)
    
    rclpy.init(args=unknown)
    node = BenchmarkNode(args.trials, args.mode)
    
    try:
        node.run_benchmark()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
