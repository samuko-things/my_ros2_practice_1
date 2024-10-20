- build ros2 packages with colcon:
  ```shell
  colcon build #build all packages in the workspace

  colcon build --packages-select <package_name>

  colcon build --packages-select <package_name> --symlink-install # good for debugging
  ```

- create python package:
  ```shell
  ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy
  ```

- create cpp package:
  ```shell
  ros2 pkg create <package_name> --build-type ament_cmake --dependencies rclcpp
  ```

- python3 interpreter line for python nodes:
  ```shell
  #!/usr/bin/env python3
  ```

- make the python file executable:
  ```shell
  chmod +x <python_file_name>
  ```

- ros2 include path for vscode:
  ```shell
  /opt/ros/<distro>/include/**
  ```

- run ros2 node:
  ```shell
  ros2 run <package_name> <executable_name>
  ```

- view all current ros2 node:
  ```shell
  ros2 node list

  ros2 node info /<node_name>
  ```

- rename a ros2 node at runtime:
  ```shell
  ros2 run <package_name> <executable_name> --ros-args --remap __node:=<new_node_name>
  ```

- install turtlesim package:
  ```shell
  sudo apt install ros-<distro>-turtlesim
  ```

- start turtle sim:
  ```shell
  ros2 run turtlesim turtlesim_node

  ros2 run turtlesim turtle_teleop_key
  ```

- list and show interface types - msg, srv, action:
  ```shell
  ros2 interface list

  ros2 interface show <interface_name> # e.g ros2 interface show example_interfaces/msg/String
  ```

- ros2 topic commandline tools:
  ```shell
  ros2 topic list

  ros2 topic info <topic_name> #get the datatype (interface) of the topic

  ros2 topic echo <topic_name> #get the actual data published on the topic

  ros2 topic hz <topic_name> #get the frequency at which data is beaing published on the topic 

  ros2 topic bw <topic_name> #get the band width the topic 
  ```

- publish topic from command line:
  ```shell
  ros2 topic pub -r <pub_freq> <topic> <interface(msg) type> <data>

  # ros2 topic pub -r 10 /robot_news example_interfaces/msg/String "{data: 'Hello World'}"
  ```

- remap a ros2 topic at runtime:
  ```shell
  ros2 run <package_name> <executable_name> --ros-args --remap <old_topic_name>:=<new_topic_name>

  ros2 run <package_name> <executable_name> --ros-args -r __node:=<new_node_name> -r <old_topic_name>:=<new_topic_name>
  ```

- ros2 service commandline tools:
  ```shell
  ros2 service list

  ros2 service type <service_name> #get the datatype (interface) of the service
  ```

- call service client from command line:
  ```shell
  ros2 service call <service> <interface(srv) type> <request_data>

  # ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"
  ```

- remap a ros2 service at runtime:
  ```shell
  ros2 run <package_name> <executable_name> --ros-args --remap <current_service_name>:=<new_service_name>

  ros2 run <package_name> <executable_name> --ros-args -r __node:=<new_node_name> -r <current_service_name>:=<new_service_name>
  ```

- working with parameters from command line:
  ```shell
  ros2 param list #list running nodes and the parameters assosiated with them

  ros2 param get <node_name> <param_name>

  ros2 run <package_name> <node_name> --ros-args -p <param_name>:=<param_val>
  ```

- launch files in ros2:
  ```shell
  ros2 launch <package_bringup> <name_of_the_launch_file>
  ```

- working with ros2 bag (cd into the folder you want to store the bag file):
  ```shell
  ros2 bag record <topic_name>

  ros2 bag record <topic_name> -o <file_name> # record a topic and store it in file_name
  ros2 bag record <topic_name1> <topic_name2> <topic_name3> -o <file_name> # record more than one topic and store them in file_name
  ros2 bag record -a -o <file_name> # record all topics running and store them in file_name

  ros2 bag info <bag_file_name> # view the info of a bag file

  ros2 bag play <bag_file_name> # play topic information in a bag file
  ```