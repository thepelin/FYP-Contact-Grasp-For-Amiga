#!/usr/bin/env python
import rospy
from ctypes import * # convert float to uint32
from std_msgs.msg import Header, Float64MultiArray
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import translation_from_matrix, quaternion_from_matrix
from amiga_moveit_wrapper.srv import *
import transforms3d as t3d
import numpy as np



def callback(msg):
    rospy.init_node('listener', anonymous=True)
    global transform_stamped
    global transform_stamped2
    tf_buffer = tf2_ros.Buffer()  # Create a buffer to store transforms

    angle_x = np.pi/2
    angle_z = np.pi

    didi = True
    data = msg.data
    # Reshape the data into desired shape
    transformation_matrix = np.array(data).reshape((4, 4))

    # Print the reshaped data
    print("Reshaped data:")
    print(transformation_matrix)
    print("")

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


    # Set the translation components of the pose
    stamped_pose_msg.pose.position.x = translated_vector[0]
    stamped_pose_msg.pose.position.y = translated_vector[1]
    stamped_pose_msg.pose.position.z = translated_vector[2]

    # Set the rotation components of the pose
    stamped_pose_msg.pose.orientation.w = quaternion_v1[0]
    stamped_pose_msg.pose.orientation.x = quaternion_v1[1]
    stamped_pose_msg.pose.orientation.y = quaternion_v1[2]
    stamped_pose_msg.pose.orientation.z = quaternion_v1[3]

    print(stamped_pose_msg)
    rate = rospy.Rate(10.0)
    idx = 1
    while not rospy.is_shutdown() and idx:
        try:
            stamped_transform01 = tf_buffer.lookup_transform("base_link","zed2_left_camera_optical_frame",rospy.Time(0))
            print(stamped_transform01)
            idx = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            print(idx)
            idx += 1
            continue
    idx = 1
    while not rospy.is_shutdown() and idx:
        try:
            stamped_transform02 = tf_buffer.lookup_transform("amiga_gripper_palm","zed2_left_camera_optical_frame",rospy.Time(0))
            print(stamped_transform02)
            idx = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            print(idx)
            print("problem")
            idx += 1
            continue
    # pose_out = do_transform_pose(stamped_pose_msg,stamped_transform01)
    pose_out = stamped_pose_msg
    pose_out2 = do_transform_pose(stamped_pose_msg,stamped_transform02)

    print("POSE OUT")
    print(pose_out)
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


idx = 0

def listener():
    while not rospy.is_shutdown():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        # tf_broadcaster = tf2_ros.TransformBroadcaster()
        global idx
        global transform_stamped
        global transform_stamped2
        rospy.init_node('listener', anonymous=True)
        rate = rospy.Rate(10)  # 10 Hz

        rospy.Subscriber("/transformation_matrix", Float64MultiArray, callback)
        rate.sleep()

        # tf_buffer = tf2_ros.Buffer()  # Create a buffer to store transforms
        # tf_listener = tf2_ros.TransformListener(tf_buffer)  # Create a listener to receive transforms
        # # Set the timestamp of the transform

        # # Publish the transform
        # idx = idx+1
        # print(idx)
        # if (idx%10) == 0:
        #     print(idx)
        #     print(transform_stamped)
        #     print(transform_stamped2)
        # if transform_stamped.child_frame_id != "":
        #     print("here")
        #     transform_stamped.header.stamp = rospy.Time.now()
        #     transform_stamped2.header.stamp = rospy.Time.now()
        #     tf_broadcaster.sendTransform(transform_stamped)
        #     tf_broadcaster.sendTransform(transform_stamped2)
        # rate.sleep()

        # Sleep to maintain the desired publishing rate
        # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    global transform_stamped
    global transform_stamped2
    transform_stamped = TransformStamped()
    transform_stamped2 = TransformStamped()
    listener()
