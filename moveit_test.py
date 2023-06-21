import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_it_test',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm_no_gripper")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print(planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print(eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print(robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:

print(robot.get_current_state())

pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.75
pose_goal.position.y = -0.026776864739967838
pose_goal.position.z = 0.43896695727988044
pose_goal.orientation.x =  0.6882475077775444
pose_goal.orientation.y = 0.7254759333041886
pose_goal.orientation.z = -0.00013734067114638915
pose_goal.orientation.w = 0.00013918317256778553
group.set_pose_target(pose_goal)

plan = group.plan()

# Get the current pose
current_pose = group.get_current_pose().pose

# Print the current pose
print("Current Pose:")
print(current_pose)
print("\n\nPLAN\n\n")
print(plan)

print("PLAN TRUE OR NOT:",plan[0])
if plan[0]:
    group.execute(plan[1])