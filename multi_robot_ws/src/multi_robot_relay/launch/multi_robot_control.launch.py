from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get config directory
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config'
    )

    return LaunchDescription([
        # Start rosapi node (provides ROS service information)
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen'
        ),

        # Start rosbridge server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
            }],
            output='screen'
        ),

        # Domain Bridge for Robot 1 (domain 1 <-> domain 0)
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_robot1',
            arguments=[os.path.join(config_dir, 'domain_bridge_robot1.yaml')],
            output='screen'
        ),

        # Domain Bridge for Robot 2 (domain 2 <-> domain 0)
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_robot2',
            arguments=[os.path.join(config_dir, 'domain_bridge_robot2.yaml')],
            output='screen'
        ),

        # Domain Bridge for Robot 3 (domain 3 <-> domain 0)
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_robot3',
            arguments=[os.path.join(config_dir, 'domain_bridge_robot3.yaml')],
            output='screen'
        ),
    ])
