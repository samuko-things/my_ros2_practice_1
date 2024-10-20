from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen',
    )

    turtle_move_to_goal_node = Node(
        package='my_turtlesim_practice',
        executable='turtle_move_to_goal',
        output='screen',
    )

    turtle_add_goal_pose_node = Node(
        package='my_turtlesim_practice',
        executable='turtle_add_goal_pose',
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the nodes to the launch description
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_move_to_goal_node)
    ld.add_action(turtle_add_goal_pose_node)

    return ld      # return (i.e send) the launch description for excecution