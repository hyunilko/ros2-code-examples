# ros2-examples

The following tutorials and examples are updated for **ROS 2 Bouncy Bolson** (Released on 2 July 2018).

## ROS2 CLI (command line interface)

Note that these commands comes from a Python package. So if you have disabled them (i.e. when cross-compiling) they will not be available.

To see a list of available commands

    $ ros2 --help

Print a list of <package_name> <executable_name>

    $ ros2 pkg executabels

Run a ROS2 node

    $ ros2 run <package_name> <executable_name>


## Running the examples

Source your ROS2 SDK, then build these packages

    $ mkdir -p ws/src
    $ cd ws/src
    $ ln -s ../../src/* .
    $ cd ..    
    $ colcon build
    $ source install/local_setup.sh


Run the following command each in a separate terminal

#### Publisher/Subscriber test

    $ ros2 run simple_publisher publisher_main
    $ ros2 run simple_subscriber subscriber_main

#### Service/Client test

    $ ros2 run simple_service service_main
    $ ros2 run simple_client client_main

Optional: Run also Rviz to visualize the data 

    $ rviz2

#### Time Synchronization message filters test

    $ ros2 run simple_publisher publisher_main
    $ ros2 run simple_time_sync time_sync_main

#### ROS2 Logger utils

    $ ros2 run logger_verbosity logger_main

#### Parameter servers

    $ ros2 run simple_parameters parameters_main
    $ ros2 run simple_parameters reader_main






## Repository content

 - The `src` directory contains source code which uses the most common ROS2 API.
 - The `dockers` directory contains Dockerfiles for creating different ROS2 developer environment.
 - The `external_build_tools` directory contains instructions for building ROS2 packages using CMake or Scons.
 - `installing_ros2` contains instructions for compiling ROS2 from sources.
 - `cross_compiling_ros2` contains instructions for cross-compiling ROS2 for ARM architectures.
 - `build_ros2_package` contains instructions for creating a ROS2 workspace and compiling packages. 
 - `resources` contains useful online documentation on ROS2.


