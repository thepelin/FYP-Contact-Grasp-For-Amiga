#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def spawn_robot():
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        robot_name = 'robot'  # Set the name for your robot

        # Retrieve the URDF from the ROS parameter
        robot_urdf = rospy.get_param('/robot_description')

        # Set the spawn position and orientation
        pose = Pose()
        pose.position.x = -0.6
        pose.position.y = 0.0
        pose.position.z = 0.15
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        # Call the spawn_model service to spawn the robot
        spawn_model(robot_name, robot_urdf, '', pose, 'world')

        rospy.loginfo("Robot spawned successfully!")
    except rospy.ServiceException as e:
        rospy.logerr("Robot spawn service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node('spawn_robot_node')

    try:
        spawn_robot()
    except rospy.ROSInterruptException:
        pass
