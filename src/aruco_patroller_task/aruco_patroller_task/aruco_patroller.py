#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import numpy as np

# --- KONSTANTE STANJA ---
PATROLLING = 0          
APPROACHING_MARKER = 1  
WAITING_AT_MARKER = 2   
CANCELING_PATROL = 3    

class ArucoPatroller(Node):
    def __init__(self):
        super().__init__("aruco_patroller")

        # --- PARAMETRI ---
        self.declare_parameter("stop_distance", 1.0) 
        self.stop_distance = self.get_parameter("stop_distance").value
        
        self.declare_parameter("wait_time", 2.0) 
        self.wait_time = self.get_parameter("wait_time").value

        self.declare_parameter("decision_timeout", 10.0)
        self.decision_timeout = self.get_parameter("decision_timeout").value
        
        # Ispod koje udaljenosti radimo korekciju točke
        self.declare_parameter("recalculation_distance", 0.3)
        self.recalculation_distance = self.get_parameter("recalculation_distance").value

        # --- NAVIGATOR ---
        self.nav = BasicNavigator()
        self.callback_group = ReentrantCallbackGroup()

        # --- TOČKE PATROLE ---
        self.patrol_waypoints = [
            self.create_pose_msg(3.0, 0.0, 0.0),
            self.create_pose_msg(4.2, 2.7, 1.57),
            self.create_pose_msg(9.2, 8.2, 0.0), 
            self.create_pose_msg(10.8, 3.1, 3.14), 
            self.create_pose_msg(6.5, -1.9, 3.14), 
            self.create_pose_msg(-1.0, 0.0, 0.0) 
        ]

        # --- SUBSCRIBERS ---
        self.create_subscription(
            PoseStamped,
            '/detected_marker_pose',
            self.marker_detected_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # --- VARIJABLE ---
        self.state = PATROLLING
        self.visited_locations = [] 
        self.current_robot_pose = None
        self.waypoint_index = 0
        self.wait_start_time = None       
        self.approach_start_time = None   
        self.trigger_new_goal = False
        self.wait_after_failure = self.get_clock().now()

        self.saved_approach_goal = None   
        self.current_target_marker_loc = None 
        
        self.last_update_time = self.get_clock().now()

        self.get_logger().info("Waiting for Nav2...")
        self.nav.waitUntilNav2Active()
        self.start_patrol()
        self.state = PATROLLING
        self.timer = self.create_timer(0.5, self.control_loop, callback_group=self.callback_group)

    def create_pose_msg(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        q = quaternion_from_euler(0, 0, yaw)
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        return p

    def pose_callback(self, msg):
        self.current_robot_pose = msg.pose.pose

    def marker_detected_callback(self, msg: PoseStamped):
        if self.current_robot_pose is None:
            return
        
        if self.get_clock().now() - self.wait_after_failure < Duration(seconds=5.0): 
            return

        rx = self.current_robot_pose.position.x
        ry = self.current_robot_pose.position.y
        mx = msg.pose.position.x
        my = msg.pose.position.y

        # Distanca od robota do detektiranog markera
        dist_robot_marker = math.hypot(mx - rx, my - ry)
        
        # Ako je marker predaleko, ignoriraj jer su problematicni 
        # Možda se i može sada ovo ukloniti ali ne mogu više testirati, 8 sati je dovoljno i bojim se
        if dist_robot_marker > 10.0:
            return

        is_current_target = False
        if self.current_target_marker_loc is not None:
            ctx, cty = self.current_target_marker_loc
            if math.hypot(mx - ctx, my - cty) < 2.0:
                is_current_target = True

        if not is_current_target:
            for vx, vy in self.visited_locations:
                if math.hypot(mx - vx, my - vy) < 2.0: 
                    self.get_logger().info(f"Ignoring visited marker at: ({mx:.2f}, {my:.2f})",throttle_duration_sec=1.0)
                    return

        # --- IZRAČUN CILJA ---
        q = [msg.pose.orientation.x, msg.pose.orientation.y, 
             msg.pose.orientation.z, msg.pose.orientation.w]
        
        rot_matrix = quaternion_matrix(q)
        normal_x = rot_matrix[0][2]
        normal_y = rot_matrix[1][2]
        
        target_x = mx + self.stop_distance * normal_x
        target_y = my + self.stop_distance * normal_y
        
        # Orijentacija robota (gledaj u marker)
        target_yaw = math.atan2(-normal_y, -normal_x)
        
        new_approach_goal = self.create_pose_msg(target_x, target_y, target_yaw)

        # =========================================================
        # SCENARIJ: PRVA DETEKCIJA (NORMALNA PATROLA)
        # =========================================================
        if self.state == PATROLLING:
            self.get_logger().info(f"Marker detected at {dist_robot_marker:.2f}m! Going to ({target_x:.2f}, {target_y:.2f})")
            
            self.saved_approach_goal = new_approach_goal
            self.current_target_marker_loc = (mx, my)
            self.visited_locations.append((mx, my))
            
            self.approach_start_time = self.get_clock().now()
            self.state = CANCELING_PATROL
            # self.nav.cancelTask()
            # Check if a goal handle exists and is active
            if hasattr(self.nav, 'goal_handle') and self.nav.goal_handle is not None:
                # cancel_goal_async returns a Future without attempting to spin
                future = self.nav.goal_handle.cancel_goal_async()
                future.add_done_callback(self._cancel_done_callback)


        elif (self.state == APPROACHING_MARKER or self.state == CANCELING_PATROL):
            
            if self.approach_start_time is None: return
            if self.current_target_marker_loc is None: return

            elapsed = (self.get_clock().now() - self.approach_start_time).nanoseconds / 1e9
            
            if is_current_target:
                current_tx, current_ty = self.current_target_marker_loc
                marker_err = math.hypot(mx - current_tx, my - current_ty)
                
                if marker_err > self.recalculation_distance:
                    now = self.get_clock().now()
                    if (now - self.last_update_time).nanoseconds / 1e9 > 1.0:
                        self.get_logger().info(f"Goal correction...", throttle_duration_sec=2.0)
                        
                        self.saved_approach_goal = new_approach_goal
                        self.current_target_marker_loc = (mx, my)
                        self.last_update_time = now
                        
                        self.trigger_new_goal = True 
                return
            
            if elapsed < self.decision_timeout:
                current_tx, current_ty = self.current_target_marker_loc
                dist_to_current = math.hypot(current_tx - rx, current_ty - ry)
                dist_to_new = dist_robot_marker 

                if dist_to_new < (dist_to_current - 1.0): 
                    self.get_logger().info(f"--- CLOSER MARKER ({dist_to_new:.2f}m)! Redirecting... ---")
                    
                    self.saved_approach_goal = new_approach_goal
                    
                    if self.visited_locations: self.visited_locations.pop() 
                    self.current_target_marker_loc = (mx, my)
                    self.visited_locations.append((mx, my))
                    
                    self.approach_start_time = self.get_clock().now()
                    self.last_update_time = self.get_clock().now()
                    
                    self.trigger_new_goal = True
                    
    def _cancel_done_callback(self, future):
        self.get_logger().info('Navigation task successfully canceled.')

                        

    def control_loop(self):
        self.get_logger().info(f"===================Current state: {self.state}====================", throttle_duration_sec=1.0)
        if self.trigger_new_goal and self.state == APPROACHING_MARKER:
            if self.saved_approach_goal:
                self.saved_approach_goal.header.stamp = self.get_clock().now().to_msg()
                self.nav.goToPose(self.saved_approach_goal)
                self.trigger_new_goal = False 

        is_complete = self.nav.isTaskComplete()
        
        if self.state == PATROLLING:
            if is_complete:
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.waypoint_index = (self.waypoint_index + 1) % len(self.patrol_waypoints)
                    if self.waypoint_index == 0:
                        self.visited_locations = []
                        self.get_logger().info("Trail done. Resetting markers.")
                    self.start_patrol()

        elif self.state == CANCELING_PATROL:
            if is_complete:
                self.get_logger().info("Robot stopped. Heading towards marker...")
                self.nav.goToPose(self.saved_approach_goal)
                self.state = APPROACHING_MARKER

        elif self.state == APPROACHING_MARKER:
            if is_complete:
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"Arrived! Waiting {self.wait_time}s.")
                    self.wait_start_time = self.get_clock().now()
                    self.state = WAITING_AT_MARKER
                elif result == TaskResult.FAILED:
                    self.get_logger().warn("Failed to visit marker. Continuing patrol.")
                    self.wait_after_failure = self.get_clock().now()
                    self.visited_locations.pop()
                    self.current_target_marker_loc = None
                    self.start_patrol()
                    self.state = PATROLLING
    
        elif self.state == WAITING_AT_MARKER:
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            if elapsed > self.wait_time:
                self.get_logger().info("Waiting done. Continuing patrol.")
                self.current_target_marker_loc = None
                self.start_patrol()
                self.state = PATROLLING

    def start_patrol(self):
        self.nav.goToPose(self.patrol_waypoints[self.waypoint_index])
        self.get_logger().info(f"Točka {self.waypoint_index + 1}.")
    

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPatroller()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()