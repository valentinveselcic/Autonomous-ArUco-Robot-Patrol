import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

def create_pose(navigator, x, y, yaw):
    """Pomoćna funkcija za kreiranje PoseStamped poruke"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    pose.pose.orientation.z = sy
    pose.pose.orientation.w = cy
    
    return pose

def main():
    rclpy.init()
    
    navigator = BasicNavigator()

    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    print("Nav2 is ready!")


    goals_list = [
        (3.0, 0.0, 0.0),    
        (4.2, 2.7, 0.0),   
        (9.2, 8.2, 0.0),  
        (10.8, 3.1, 3.14),    
        (6.5, -1.9, 3.14),
        (0.0, 0.0, 0.0)
    ]
    
    waypoints = []
    
    for x, y, yaw in goals_list:
        waypoints.append(create_pose(navigator, x, y, yaw))

    for pose in waypoints:
        navigator.goToPose(pose)
        while not navigator.isTaskComplete():
            pass
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Successfully visited all points!')
    elif result == TaskResult.CANCELED:
        print('Task canceled.')
    elif result == TaskResult.FAILED:
        print('Navigation failed!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()