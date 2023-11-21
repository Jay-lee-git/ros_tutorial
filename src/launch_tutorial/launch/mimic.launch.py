from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
               #  topic: input/pose를 받고 /turtle2/pose로 변환
                ('/input/pose', '/turtle2/pose'),
                # /output/cmd_vel -> turtlesim2/turtle1/cmd_vel으로 전환
                ('/output/cmd_vel', 'turtlesim2/turtle1/cmd_vel'),
            ]
            
        )
    ])