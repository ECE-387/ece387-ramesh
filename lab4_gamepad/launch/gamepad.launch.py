import launch
import launch_ros.actions

def generate_launch_description():
    """
    Launches the gamepad node from lab4_gamepad
    and the joy_node from the joy package.
    """
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='lab4_gamepad',
            executable='gamepad',
            name='gamepad_node',
            output='screen'
        ),
    ])
