import roslaunch

# Create a ROS launch parent object
parent = roslaunch.parent.ROSLaunchParent(None, [])

# Create a dictionary to store the arguments
args = {
    'initial_joint_positions': '-J amiga_arm_elbow_joint 2.8274 -J amiga_arm_shoulder_lift_joint -1.5882 -J amiga_arm_shoulder_pan_joint 1.4486 -J amiga_arm_wrist_1_joint -1.7977 -J amiga_arm_wrist_2_joint 2.9845 -J amiga_arm_wrist_3_joint -0.192',
    'random_value1': '0.0'
}

# Create a ROS launch configuration object
config = roslaunch.config.ROSLaunchConfig()

# Iterate over the arguments and add them to the configuration
for arg_name, arg_value in args.items():
    arg = roslaunch.config.ParamArg(arg_value)
    config.add_arg(roslaunch.config.Arg(arg_name, arg))

# Specify the path to your launch file
launch_file = '/path/to/your/launch_file.launch'

# Load the launch file into the configuration
roslaunch.tools.load_file(launch_file, config)

# Create a ProcessMonitor with the configuration
process_monitor = roslaunch.pmon.ProcessMonitor(parent, config)

# Start the ROS launch
process_monitor.start()

# Shutdown the ROS launch
process_monitor.shutdown()
