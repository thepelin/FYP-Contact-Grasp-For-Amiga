import sys
import moveit_commander
import moveit_msgs.msg
import rospy
from ctypes import * # convert float to uint32
from std_msgs.msg import Header, Float64MultiArray
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped, Pose
from tf2_geometry_msgs import do_transform_pose
from amiga_moveit_wrapper.srv import *
import transforms3d as t3d
from contact_graspnet_ros.srv import InferenceDatawithBB
import numpy as np



moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm_no_gripper")
list_of_booleans = []

type_dict = {
    "bowl": -0.1,
    "sink": -0.06,
    "toilet": -0.06,
    "frisbee": -0.06,
    "bottle": 0.03,
    "vase": 0.03,
    "baseball bat": -0.05,
    "carrot": -0.05
}

rospy.init_node('com_transtopose', anonymous=True)
tf_buffer = tf2_ros.Buffer()  # Create a buffer to store transforms
tf_listener = tf2_ros.TransformListener(tf_buffer)  # Create a listener to receive transforms

ans = input("which object to grab?")
rospy.wait_for_service('/contact_graspnet/bb_inference')
try:
    inferenceservice = rospy.ServiceProxy('/contact_graspnet/bb_inference', InferenceDatawithBB)
    inferencedata = inferenceservice(ans)
    print("got the inference data")
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
inferencevector = inferencedata.out

def create_matrices(input_list, rows, columns):
    matrices = []
    matrix = []

    for i, value in enumerate(input_list):
        matrix.append(value)
        if (i + 1) % columns == 0:
            matrices.append(matrix)
            matrix = []

    return [matrices[i:i+rows] for i in range(0, len(matrices), rows)]

transformation_matrix_list = create_matrices(inferencevector,4,4)


count_vis = 20
for transformation_matrix in transformation_matrix_list:
    
    if count_vis == 0:
        break
    else:
        count_vis = count_vis - 1
        print(count_vis)
    # Define the rotation angles
    # angle_x = 3.2 * np.pi / 8
    angle_x = np.pi/2
    angle_z = np.pi
    # angle_x = np.pi/2
    # angle_z = 0

    # Construct the rotation matrices
    rot_x = np.array([[1, 0, 0, 0],
                    [0, np.cos(angle_x), -np.sin(angle_x), 0],
                    [0, np.sin(angle_x), np.cos(angle_x), 0],
                    [0, 0, 0, 1]])

    rot_z = np.array([[np.cos(angle_z), -np.sin(angle_z), 0, 0],
                    [np.sin(angle_z), np.cos(angle_z), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])



    # Compute the final transformation matrix
    displacement = np.dot(rot_z, rot_x)
    displacement[0][3] = 0
    displacement[1][3] = 0
    displacement[2][3] = 0 #type_dict[ans]
    # displacement = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]



    T, R, Z, S = t3d.affines.decompose(np.dot(transformation_matrix,displacement))

    translated_vector1 = T

    ai,aj,ak = t3d.euler.mat2euler(R,axes='rzyx')
    quaternion_v1 = t3d.quaternions.mat2quat(R)

    # Create a Stamped Pose message
    stamped_pose_msg1 = PoseStamped()
    #stamped_transform = TransformStamped()

    # Set the header of the message
    stamped_pose_msg1.header.frame_id = "zed2_left_camera_optical_frame"

    stamped_pose_msg1.header.stamp = rospy.Time.now()

    # Set the translation components of the pose
    stamped_pose_msg1.pose.position.x = translated_vector1[0]
    stamped_pose_msg1.pose.position.y = translated_vector1[1]
    stamped_pose_msg1.pose.position.z = translated_vector1[2]

    # Set the rotation components of the pose
    stamped_pose_msg1.pose.orientation.w = quaternion_v1[0]
    stamped_pose_msg1.pose.orientation.x = quaternion_v1[1]
    stamped_pose_msg1.pose.orientation.y = quaternion_v1[2]
    stamped_pose_msg1.pose.orientation.z = quaternion_v1[3]
    print(stamped_pose_msg1)
    collusion_line = TransformStamped()
    # Set the frame ID and child frame ID
    collusion_line.header.frame_id = stamped_pose_msg1.header.frame_id 
    collusion_line.child_frame_id = 'collusion_line'
    collusion_line.header.stamp = rospy.Time.now()

    # Set the transform translation
    collusion_line.transform.translation.x = stamped_pose_msg1.pose.position.x
    collusion_line.transform.translation.y = stamped_pose_msg1.pose.position.y
    collusion_line.transform.translation.z = stamped_pose_msg1.pose.position.z

    # Set the transform rotation as a quaternion
    collusion_line.transform.rotation.w = stamped_pose_msg1.pose.orientation.w
    collusion_line.transform.rotation.x = stamped_pose_msg1.pose.orientation.x
    collusion_line.transform.rotation.y = stamped_pose_msg1.pose.orientation.y
    collusion_line.transform.rotation.z = stamped_pose_msg1.pose.orientation.z
    

    rate = rospy.Rate(10.0)
    idx = 1
    while not rospy.is_shutdown() and idx:
        try:
            stamped_transform = tf_buffer.lookup_transform("base_link","zed2_left_camera_optical_frame",rospy.Time(0))
            print(stamped_transform)
            idx = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            print(idx)
            idx += 1
            continue
    # pose_out = do_transform_pose(stamped_pose_msg,stamped_transform01)
    pose_out1 = do_transform_pose(stamped_pose_msg1,stamped_transform)

    print("POSE OUT")
    print(pose_out1)
 
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    transform_stamped1 = TransformStamped()

    # Set the frame ID and child frame ID
    transform_stamped1.header.frame_id = pose_out1.header.frame_id 
    transform_stamped1.child_frame_id = 'goal_pose_cg4'
    transform_stamped1.header.stamp = rospy.Time.now()

    # Set the transform translation
    transform_stamped1.transform.translation.x = pose_out1.pose.position.x
    transform_stamped1.transform.translation.y = pose_out1.pose.position.y
    transform_stamped1.transform.translation.z = pose_out1.pose.position.z

    # Set the transform rotation as a quaternion
    transform_stamped1.transform.rotation.w = pose_out1.pose.orientation.w
    transform_stamped1.transform.rotation.x = pose_out1.pose.orientation.x
    transform_stamped1.transform.rotation.y = pose_out1.pose.orientation.y
    transform_stamped1.transform.rotation.z = pose_out1.pose.orientation.z

    # Create a Stamped Pose message
    pose_orientation_check = PoseStamped()
    #stamped_transform = TransformStamped()

    # Set the header of the message
    pose_orientation_check.header.frame_id = "base_link"

    
    pose_orientation_check.pose.position.x = 0
    pose_orientation_check.pose.position.y = 0
    pose_orientation_check.pose.position.z = 1
    pose_orientation_check.pose.orientation.x = 0
    pose_orientation_check.pose.orientation.y = 0
    pose_orientation_check.pose.orientation.z = 0
    pose_orientation_check.pose.orientation.w = 1
    
    # pose_orientation_check = do_transform_pose(pose_orientation_check,transform_stamped1)
    # if pose_orientation_check.pose.position.z > 0:
    #     print("\n\nUPSIDE DOWN\n\n")
    #     pose_orientation_check = PoseStamped()

    #     pose_orientation_check.header.frame_id = "base_link"

    #     pose_orientation_check.pose.position.x = 0
    #     pose_orientation_check.pose.position.y = 0
    #     pose_orientation_check.pose.position.z = 0
    #     pose_orientation_check.pose.orientation.x = 0
    #     pose_orientation_check.pose.orientation.y = 1
    #     pose_orientation_check.pose.orientation.z = 0
    #     pose_orientation_check.pose.orientation.w = 0
        
    #     pose_orientation_check = do_transform_pose(pose_orientation_check,transform_stamped1)
    #     pose_out1 = pose_orientation_check

    #      # Set the transform translation
    #     transform_stamped1.transform.translation.x = pose_out1.pose.position.x
    #     transform_stamped1.transform.translation.y = pose_out1.pose.position.y
    #     transform_stamped1.transform.translation.z = pose_out1.pose.position.z

    #     # Set the transform rotation as a quaternion
    #     transform_stamped1.transform.rotation.w = pose_out1.pose.orientation.w
    #     transform_stamped1.transform.rotation.x = pose_out1.pose.orientation.x
    #     transform_stamped1.transform.rotation.y = pose_out1.pose.orientation.y
    #     transform_stamped1.transform.rotation.z = pose_out1.pose.orientation.z
    # else:
    #     print("\n\nCORRECT\n\n")
    

    pose_goal = Pose()
    pose_goal.position.x = pose_out1.pose.position.x
    pose_goal.position.y = pose_out1.pose.position.y
    pose_goal.position.z = pose_out1.pose.position.z
    pose_goal.orientation.x = pose_out1.pose.orientation.x
    pose_goal.orientation.y = pose_out1.pose.orientation.y
    pose_goal.orientation.z = pose_out1.pose.orientation.z
    pose_goal.orientation.w = pose_out1.pose.orientation.w

    # rospy.wait_for_service('/amiga_moveit_wrapper/go_to_pose/async')
    # try:
    #     pose_send = rospy.ServiceProxy('/amiga_moveit_wrapper/go_to_pose/async', GoToPose)
    #     worked = pose_send(pose_out1)
    #     print(worked)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)
    
    # group.set_pose_target(pose_goal)

    # plan = group.plan()



    # print("PLAN TRUE OR NOT:",plan[0])
    rate = rospy.Rate(10)  # 10 Hz
    for index in range(10):
        print(index)
        # Set the timestamp of the transform
        transform_stamped1.header.stamp = rospy.Time.now()
        # Publish the transform
        tf_broadcaster.sendTransform(transform_stamped1)
        # Sleep to maintain the desired publishing rate
        rate.sleep()
    # if plan[0]:
    #     group.execute(plan[1])
    #     break


print(list_of_booleans)