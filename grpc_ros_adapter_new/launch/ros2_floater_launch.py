from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="grpc_ros_adapter",  
            executable="volume_disp_publisher",
            name="volume_disp_publisher",
            output="screen",
           
        ),
        Node(
            package="grpc_ros_adapter",  
            executable="larvaeCountNode",
            name="larvae_count_node",
            output="screen",
            
        ),
        Node(
            package="grpc_ros_adapter",  
            executable="GNSSSubscriber",
            name="gnss_subscriber",
            output="screen",
            
        ),
    ])
