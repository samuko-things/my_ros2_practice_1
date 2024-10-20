from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    remap_number_topic = ("number", "my_number")

    number_publisher_node = Node(
        package='my_py_practice',
        executable='number_publisher',
        name="my_number_publisher", # use to rename node
        output='screen',
        remappings=[ # use to remapp topics or services
            remap_number_topic
        ],
        parameters=[ # set parameters (without yaml file)
            {"number_to_publish": 4},
            {"publish_frequency": 5.0}
        ]
    )

    number_counter_node = Node(
        package='my_cpp_practice',
        executable='number_counter',
        name="my_number_counter", # use to rename node
        output='screen',
        remappings=[ # use to remapp topics or services
            remap_number_topic,
            ("number_count", "my_number_count")
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the nodes to the launch description
    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld      # return (i.e send) the launch description for excecution