import rospy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import os


depth_data = None
camera_info_data = None
bboxes = None
found = False

def depth_callback(depth):
    global depth_data
    depth_data = depth
    print("got depth info")

def camera_info_callback(camera_info):
    global camera_info_data
    camera_info_data = camera_info
    print("got camera info")
    if depth_data is not None:
        save_data(depth_data, camera_info_data)

def bbox_callback(msg):
    # Store the received bounding box coordinates
    print("bbox_callback")
    global bboxes
    bboxes = msg


def save_data(depth, camera_info):
    # Convert the data to numpy array of float32
    global found
    for bb in bboxes.bounding_boxes:
        if bb.Class == "bottle" or  bb.Class == "vase":
            xmin = bb.xmin-10
            xmax = bb.xmax+10
            ymin = bb.ymin-10
            ymax = bb.ymax+10
            print("x and y",xmin,xmax,ymin,ymax)
            found = True
            depth_array_origin = np.frombuffer(depth.data, dtype=np.float32)
            depth_array = depth_array_origin.copy()
            depth_array = depth_array.reshape((depth.height, depth.width))
            # depth_array[:ymin,:] = np.nan
            # depth_array[ymax:,:] = np.nan
            # depth_array[:, :xmin] = np.nan
            # depth_array[:, xmax:] = np.nan
            # print(len(np.unique(depth_array)))
            # Reshape the depth array to match the image dimensions
            #depth_array = depth_array.reshape((depth.height, depth.width))

            # Save the depth array and camera info as .npz file
            BASE_DIR = os.path.dirname(os.path.abspath(__file__))
            file_path = BASE_DIR + "/data2.npz"
            print(file_path)
            np.savez(file_path, depth=depth_array, K=camera_info.K)

            rospy.loginfo("Depth image and camera info saved")
            print("saved")

def main():
    rospy.init_node('depth_data_subscriber')
    rospy.Subscriber('/zed2_node/depth/image_raw', Image, depth_callback)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, bbox_callback)
    rospy.Subscriber('/zed2_node/depth/camera_info', CameraInfo, camera_info_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

# import rospy
# import numpy as np
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs import point_cloud2

# def point_cloud_callback(msg):
#     # Convert sensor_msgs/PointCloud2 to NumPy array
#     point_cloud_arr = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     point_cloud_arr = np.array(list(point_cloud_arr))

#     # Create a dictionary with the 'xyz' key and the point cloud array as the value
#     data_dict = {'xyz': point_cloud_arr}

#     # Save the dictionary as a .npz file
#     np.savez('point_cloud_data.npz', **data_dict)
#     print("saved")

# # Initialize the ROS node
# rospy.init_node('point_cloud_subscriber')

# # Subscribe to the point cloud topic
# rospy.Subscriber('/zed2_node/point_cloud/cloud_registered', PointCloud2, point_cloud_callback)

# # Spin the ROS node to receive messages
# rospy.spin()
