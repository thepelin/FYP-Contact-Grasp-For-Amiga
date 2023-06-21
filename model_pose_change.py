import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist


def change_model_pose():
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Set the name of the model to change its pose
        model_name = 'robot'  # Set the name of your model

        # Set the new pose for the model
        pose = Pose()
        pose.position.x = 0.0  # Set the desired position in the x-axis
        pose.position.y = 0.0  # Set the desired position in the y-axis
        pose.position.z = 0.15  # Set the desired position in the z-axis
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        # Create the model state message
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = pose

        # Call the set_model_state service to change the model's pose
        set_model_state(model_state)

        rospy.loginfo("Model pose changed successfully!")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to change model pose: %s" % e)


if __name__ == '__main__':
    rospy.init_node('change_model_pose_node')

    try:
        change_model_pose()
    except rospy.ROSInterruptException:
        pass