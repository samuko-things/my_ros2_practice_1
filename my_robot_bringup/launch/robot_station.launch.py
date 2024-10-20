from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    robot_names = ["Giskard", "BB8", "Daneel", "Jander", "C3PO"]
    robot_station_nodes = []

    for robot_name in robot_names:
        robot_station_nodes.append(Node(
            package='my_cpp_practice',
            executable='robot_news_station',
            name="robot_news_station_" + robot_name, # use to rename node
            output='screen',
            parameters=[ # set parameters (without yaml file)
                {"robot_name": robot_name}
            ]
        ))

    smart_phone_node = Node(
        package='my_cpp_practice',
        executable='smart_phone',
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the nodes to the launch description
    for robot_station_node in robot_station_nodes:
        ld.add_action(robot_station_node)
    ld.add_action(smart_phone_node)

    return ld      # return (i.e send) the launch description for excecution