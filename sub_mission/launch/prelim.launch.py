import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='sub_control', node_executable='service', output='screen',
            name=[
                'sub_control_service'
            ],
            parameters=[
                {"SIM": True}
            ]
        ),
        launch_ros.actions.Node(
            package='sub_mission', node_executable='prelim_node', output='screen',
            name=[
                'prelim_node'
            ]
        ),
    ])
