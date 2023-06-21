import rospy
from sensor_msgs.msg import CameraInfo, PointCloud2

import numpy as np
import pcl
import sensor_msgs.point_cloud2 as pcl2



class PointCloudFilterNode(object):
    def __init__(self):
        rospy.init_node('point_cloud_filter_node')

        self.camera_info_sub = rospy.Subscriber('/zed2_node/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.filtered_pc_pub = rospy.Publisher('/filtered_point_cloud_topic', PointCloud2, queue_size=10)

        self.pc_sub = rospy.Subscriber('/zed2_node/point_cloud/cloud_registered', PointCloud2, self.point_cloud_callback)

        # self.bbox_sub = rospy.Subscriber('/bounding_box_topic', BoundingBox, self.bbox_callback)


        # Initialize variables
        # self.camera_info = None
        # self.bbox = None

    def camera_info_callback(self, msg):
        # Store the received camera info
        self.camera_info = msg

    # def bbox_callback(self, msg):
    #     # Store the received bounding box coordinates
    #     self.bbox = msg

    def point_cloud_callback(self, msg):
        if self.camera_info is None or self.bbox is None:
            # Wait until both camera info and bounding box are available
            return

        # Convert the received point cloud message to a PCL point cloud object
        point_cloud = pcl.PointCloud()
        point_cloud.from_msg(msg)

        # Extract the K matrix from the camera info message
        K = np.reshape(self.camera_info.P, (3, 4))

        # Get the bounding box coordinates
        bbox_xmin = self.bbox.xmin = 400
        bbox_xmax = self.bbox.xmax = 1000
        bbox_ymin = self.bbox.ymin = 400
        bbox_ymax = self.bbox.ymax = 1000

        # Filter the point cloud based on the bounding box
        filtered_cloud = pcl.PointCloud()
        indices = []
        for point in point_cloud:
            # Convert the 3D point to the 2D image plane using the K matrix
            uvw = np.dot(K, [point[0], point[1], point[2], 1.0])
            u = uvw[0] / uvw[2]
            v = uvw[1] / uvw[2]

            # Check if the point lies within the bounding box
            if bbox_xmin <= u <= bbox_xmax and bbox_ymin <= v <= bbox_ymax:
                filtered_cloud.append(point)
                indices.append(point_cloud.index(point))

        # Convert the filtered point cloud to a ROS message
        filtered_pc_msg = pcl2.create_cloud_xyz32(msg.header, filtered_cloud.to_list())

        # Publish the filtered point cloud
        self.filtered_pc_pub.publish(filtered_pc_msg)

# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('converter', anonymous=True)
#     xmin = 800
#     xmax = 1000
#     ymin = 400
#     ymax = 1000
#     #rospy.Subscriber("/zed2_node/rgb/camera_info", CameraInfo, getK)
#     points = rospy.wait_for_message("/zed2_node/point_cloud/cloud_registered", PointCloud2)
#     #pub = rospy.Publisher('/output',PointCloud,queue_size=1)
#     print("here", type(points))

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

if __name__ == '__main__':
    node = PointCloudFilterNode()
    rospy.spin()

