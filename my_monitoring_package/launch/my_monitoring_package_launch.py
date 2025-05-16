from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Sensor 1 Node
        Node(
            package='my_monitoring_package',
            executable='sensor',
            name='sensor1_node',
            parameters=[
                {'sensor_name': 'sensor1'}
            ],
        ),

        # Sensor 2 Node
        Node(
            package='my_monitoring_package',
            executable='sensor',
            name='sensor2_node',
            parameters=[
                {'sensor_name': 'sensor2'}
            ],
        ),

        # State Monitor Node
        Node(
            package='my_monitoring_package',
            executable='state_monitor',
            name='state_monitor',
            parameters=[
                {'sensors': ['sensor1', 'sensor2'], 'timeout': 5.0}
            ],
        ),

        # Error Handler Node
        Node(
            package='my_monitoring_package',
            executable='error_handler',
            name='error_handler',
        ),
    ])
