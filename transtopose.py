import sys
import moveit_commander
import moveit_msgs.msg
import rospy
from ctypes import * # convert float to uint32
from std_msgs.msg import Header, Float64MultiArray
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped, Pose
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import translation_from_matrix, quaternion_from_matrix
from amiga_moveit_wrapper.srv import *
import transforms3d as t3d
import numpy as np


rospy.init_node('talker', anonymous=True)
tf_buffer = tf2_ros.Buffer()  # Create a buffer to store transforms
tf_listener = tf2_ros.TransformListener(tf_buffer)  # Create a listener to receive transforms

transformation_matrix = [[ 0.8762379 ,  0.0306624 , -0.48090222,  0.10491548],
       [-0.05524716,  0.9977853 , -0.03704529,  0.20511378],
       [ 0.4787012 ,  0.05902897,  0.8759913 ,  0.5174361 ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]
# transformation_matrix = [[1.0, 0.0, 0.0, 0.0],
#                         [0.0, 1.0, 0.0, 0.0],
#                         [0.0, 0.0, 1.0, 0.0],
#                         [0.0, 0.0, 0.0, 1.0]]

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
displacement[2][3] = 0


T, R, Z, S = t3d.affines.decompose(displacement)

translated_vector2 = T

ai,aj,ak = t3d.euler.mat2euler(R,axes='rzyx')
quaternion_v4 = t3d.quaternions.mat2quat(R)

T, R, Z, S = t3d.affines.decompose(np.dot(transformation_matrix,displacement))

translated_vector3 = T

ai,aj,ak = t3d.euler.mat2euler(R,axes='rzyx')
quaternion_v5 = t3d.quaternions.mat2quat(R)


# transformation_matrix = np.dot(displacement,transformation_matrix)
print("trans:", transformation_matrix)

# Create a Stamped Pose message
stamped_pose_msg = PoseStamped()
#stamped_transform = TransformStamped()

# Set the header of the message
stamped_pose_msg.header.frame_id = "zed2_left_camera_optical_frame"

stamped_pose_msg.header.stamp = rospy.Time.now()

# Decompose the transformation matrix

T, R, Z, S = t3d.affines.decompose(transformation_matrix)

translated_vector = T

ai,aj,ak = t3d.euler.mat2euler(R,axes='rzyx')
quaternion_v1 = t3d.euler.euler2quat(ai,aj,ak,axes='rzyx')
quaternion_v2 = t3d.quaternions.mat2quat(R)

print("3df")
print("Translations:", translated_vector)
print("Quaternion V1:", quaternion_v1)
print("Quaternion V2:", quaternion_v2)

# scale, shear, angles, trans, persp = decompose_matrix(transformation_matrix)
# R1 = euler_matrix(*angles)
# euler_vals = euler_from_matrix(R1)
# fe = quaternion_from_euler(euler_vals[0],euler_vals[1],euler_vals[2])
# print("qfe:",qfe)
# print("euler", R1)
# print("quaternion_rot",quaternion_rot)
# print("scale, shear, angles, trans, persp", scale, shear, angles, trans, persp)

# stamped_pose_msg.pose.position.x = 1
# stamped_pose_msg.pose.position.y = 0
# stamped_pose_msg.pose.position.z = 0.5

# Set the rotation components of the pose
# stamped_pose_msg.pose.orientation.x = 0
# stamped_pose_msg.pose.orientation.y = 0
# stamped_pose_msg.pose.orientation.z = 0
# stamped_pose_msg.pose.orientation.w = 0

# Set the translation components of the pose
stamped_pose_msg.pose.position.x = translated_vector[0]
stamped_pose_msg.pose.position.y = translated_vector[1]
stamped_pose_msg.pose.position.z = translated_vector[2]

# Set the rotation components of the pose
stamped_pose_msg.pose.orientation.w = quaternion_v1[0]
stamped_pose_msg.pose.orientation.x = quaternion_v1[1]
stamped_pose_msg.pose.orientation.y = quaternion_v1[2]
stamped_pose_msg.pose.orientation.z = quaternion_v1[3]

# Create a Stamped Pose message
stamped_pose_msg2 = PoseStamped()
#stamped_transform = TransformStamped()

# Set the header of the message
stamped_pose_msg2.header.frame_id = "zed2_left_camera_optical_frame"

stamped_pose_msg2.header.stamp = rospy.Time.now()

# Set the translation components of the pose
stamped_pose_msg2.pose.position.x = translated_vector3[0]
stamped_pose_msg2.pose.position.y = translated_vector3[1]
stamped_pose_msg2.pose.position.z = translated_vector3[2]

# Set the rotation components of the pose
stamped_pose_msg2.pose.orientation.w = quaternion_v5[0]
stamped_pose_msg2.pose.orientation.x = quaternion_v5[1]
stamped_pose_msg2.pose.orientation.y = quaternion_v5[2]
stamped_pose_msg2.pose.orientation.z = quaternion_v5[3]

print(stamped_pose_msg)
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
pose_out = stamped_pose_msg
pose_out2 = do_transform_pose(stamped_pose_msg2,stamped_transform)

# pose_out2.header.frame_id = 'zed2_left_camera_optical_frame'
# pose_out2 = do_transform_pose(pose_out_inter,stamped_transform01)

# pose_out.pose.position.x = 1.0143775535453023
# pose_out.pose.position.y = -0.02456283745239623
# pose_out.pose.position.z =  0.34737712307981744

# # Set the transform rotation as a quaternion
# pose_out.pose.orientation.w = -0.42578945494005604
# pose_out.pose.orientation.x = 0.6911172068742855
# pose_out.pose.orientation.y = -0.5261836677315868
# pose_out.pose.orientation.z = 0.25335961445477434
print("POSE OUT")
print(pose_out2)
print("POSE OUT")
print(pose_out2)

tf_broadcaster = tf2_ros.TransformBroadcaster()
transform_stamped = TransformStamped()

# Set the frame ID and child frame ID
transform_stamped.header.frame_id = pose_out.header.frame_id 
transform_stamped.child_frame_id = 'goal_pose_pelin'
transform_stamped.header.stamp = rospy.Time.now()

# Set the transform translation
transform_stamped.transform.translation.x = pose_out.pose.position.x
transform_stamped.transform.translation.y = pose_out.pose.position.y
transform_stamped.transform.translation.z = pose_out.pose.position.z

# Set the transform rotation as a quaternion
transform_stamped.transform.rotation.w = pose_out.pose.orientation.w
transform_stamped.transform.rotation.x = pose_out.pose.orientation.x
transform_stamped.transform.rotation.y = pose_out.pose.orientation.y
transform_stamped.transform.rotation.z = pose_out.pose.orientation.z

transform_stamped2 = TransformStamped()

# Set the frame ID and child frame ID
transform_stamped2.header.frame_id = 'goal_pose_pelin'
transform_stamped2.child_frame_id = 'goal_pose_pelin2'
transform_stamped2.header.stamp = rospy.Time.now()

# Set the transform translation
transform_stamped2.transform.translation.x = translated_vector2[0]
transform_stamped2.transform.translation.y = translated_vector2[1]
transform_stamped2.transform.translation.z = translated_vector2[2]

# Set the transform rotation as a quaternion
transform_stamped2.transform.rotation.w = quaternion_v4[0]
transform_stamped2.transform.rotation.x = quaternion_v4[1]
transform_stamped2.transform.rotation.y = quaternion_v4[2]
transform_stamped2.transform.rotation.z = quaternion_v4[3]

transform_stamped3 = TransformStamped()

# Set the frame ID and child frame ID
transform_stamped3.header.frame_id = pose_out2.header.frame_id 
transform_stamped3.child_frame_id = 'goal_pose_pelin4'
transform_stamped3.header.stamp = rospy.Time.now()

# Set the transform translation
transform_stamped3.transform.translation.x = pose_out2.pose.position.x
transform_stamped3.transform.translation.y = pose_out2.pose.position.y
transform_stamped3.transform.translation.z = pose_out2.pose.position.z

# Set the transform rotation as a quaternion
transform_stamped3.transform.rotation.w = pose_out2.pose.orientation.w
transform_stamped3.transform.rotation.x = pose_out2.pose.orientation.x
transform_stamped3.transform.rotation.y = pose_out2.pose.orientation.y
transform_stamped3.transform.rotation.z = pose_out2.pose.orientation.z

print(transform_stamped3)

# rospy.wait_for_service('/amiga_moveit_wrapper/go_to_pose/async')
# try:
#     pose_send = rospy.ServiceProxy('/amiga_moveit_wrapper/go_to_pose/async', GoToPose)
#     worked = pose_send(pose_out2)
#     print(worked)
# except rospy.ServiceException as e:
#     print("Service call failed: %s"%e)

rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # Set the timestamp of the transform
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped2.header.stamp = rospy.Time.now()
    transform_stamped3.header.stamp = rospy.Time.now()
    # Publish the transform
    tf_broadcaster.sendTransform(transform_stamped)
    tf_broadcaster.sendTransform(transform_stamped2)
    tf_broadcaster.sendTransform(transform_stamped3)
    # Sleep to maintain the desired publishing rate
    rate.sleep()


# pub = rospy.Publisher('/pose_point', PointStamped, queue_size=1,latch = True)

# point_msg = PointStamped()
# point_msg.header.stamp = rospy.Time.now()  # Set the timestamp
# point_msg.header.frame_id = 'base_link'  # Set the frame ID

# # Set the position values of the point
# point_msg.point.x = pose_out.pose.position.x  # Replace with the actual x-coordinate of the pose
# point_msg.point.y = pose_out.pose.position.y  # Replace with the actual y-coordinate of the pose
# point_msg.point.z = pose_out.pose.position.z  # Replace with the actual z-coordinate of the pose
# print(point_msg)
# rate = rospy.Rate(10) 
# while not rospy.is_shutdown():
#     pub.publish(point_msg)  # Publish the point message
#     print("published:")
#     rate.sleep()