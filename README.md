# MARBLE-hackaton

# grpc_ros_adapter
* modified grpc_ros_adapter package is under grpc_ros_adapter_new

## Changes done
* added volume_disp_publisher.py - controls the bouyacy of the floater 
to run execute:
"ros2 run grpc_ros_adapter volume_disp_publisher" 

* added larvaeCountNode.py - uses NN to count the number of larvae 
-the NN model used is saved with the scripts
-it is important to have torchvision installed ("pip install torch torchvision")
to run execute:
"ros2 run  grpc_ros_adapter larvaeCountNode" 

* added GNSSSubscriber.py - takes the number of larvae, the position of the floater and plots it
to run execute:
"ros2 run grpc_ros_adapter GNSSSubscriber" 

* added ros2_floater_launch.py - launches all scripts mentioned above
to run execute:
"ros2 launch grpc_ros_adapter ros2_floater_launch.py " 

* setup.py has been modified to allow for building the package 

* befor launching the ros2_floater_launch.py it is important to have the Unity simulator running and the "ros2_server_launch.py" launched.
* number of floaters can be changed and needs to be changed in every script in the array "floater_names", these names need to match the names in the Unity simulator (current setup is 2 floater, floater and floater_2)
