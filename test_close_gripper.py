import sys
import rospy
import moveit_commander
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped

def go_to_predefined_state():
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)

    # Create a RobotCommander object#
    scene = moveit_commander.PlanningSceneInterface()
    
    robot = moveit_commander.RobotCommander()

    # Create a MoveGroupCommander object for the desired group
    group_name = "sim_gripper"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    box_pose = PoseStamped()
    box_pose.header.frame_id = "amiga_gripper_palm"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    # Set the target state
    target_state = "closed_gripper"
    touch_links = robot.get_link_names(group=group_name)
    scene.attach_box("amiga_gripper_palm", box_name, touch_links=touch_links)
    move_group.set_named_target(target_state)

    # Plan and execute the motion
    move_group.go()

if __name__ == "__main__":
    go_to_predefined_state()
