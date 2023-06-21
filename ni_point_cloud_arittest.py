import rospy
from sensor_msgs.msg import CameraInfo, PointCloud2

import numpy as np
import sensor_msgs.point_cloud2 as pcl2
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox



class PointCloudFilterNode(object):
    def __init__(self):
        rospy.init_node('point_cloud_filter_node')
        print("hey")
        self.camera_info_sub = rospy.Subscriber('/zed2_node/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.filtered_pc_pub = rospy.Publisher('/filtered_point_cloud_topic', PointCloud2, queue_size=10, latch = True)
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
        self.pc_sub = rospy.Subscriber('/zed2_node/point_cloud/cloud_registered', PointCloud2, self.point_cloud_callback)

        


        # Initialize variables
        # self.camera_info = None
        # self.bbox = None

    def camera_info_callback(self, msg):
        # Store the received camera info
        self.camera_info = msg

    def bbox_callback(self, msg):
        # Store the received bounding box coordinates
        found = False
        for bb in msg.bounding_boxes:
            if bb.Class == "bottle" or  bb.Class == "bottle":
                self.xmin = bb.xmin
                self.xmax = bb.xmax
                self.ymin = bb.ymin
                self.ymax = bb.ymax
                found = True
        if not found:
            self.xmin = 500
            self.xmax = 1000
            self.ymin = 500
            self.ymax = 1000
        print("values:",bb.xmin, bb.xmax, bb.ymin, bb.ymax)

    def point_cloud_callback(self, msg):
        if self.camera_info is None:
            # Wait until both camera info and bounding box are available
            return

        # Convert the received point cloud message to a PCL point cloud object

        # Extract the K matrix from the camera info message
        K = np.reshape(self.camera_info.P, (3, 4))

        # Get the bounding box coordinates
        # bbox_xmin = self.bbox.xmin = 400
        # bbox_xmax = self.bbox.xmax = 1000
        # bbox_ymin = self.bbox.ymin = 400
        # bbox_ymax = self.bbox.ymax = 1000

        bbox_xmin = self.xmin
        bbox_xmax = self.xmax
        bbox_ymin = self.ymin
        bbox_ymax = self.ymax
        print("cheers for the correct values")
        # Filter the point cloud based on the bounding box
        filtered_cloud = []
        i = 0
        # pcl2.read_points(msg, skip_nans=True)
        for point in pcl2.read_points(msg, skip_nans=True):
            # # Convert the 3D point to the 2D image plane using the K matrix
            uvw = np.dot(K, [point[0], point[1], point[2], 1.0])
            u = uvw[0] / uvw[2]
            v = uvw[1] / uvw[2]

            # Check if the point lies within the bounding box
            if bbox_xmin <= u <= bbox_xmax and bbox_ymin <= v <= bbox_ymax:
                a = 1
                filtered_cloud.append(point)
            i = i+1
            #filtered_cloud.append(point)
        
        print(len(filtered_cloud), i)
        print("values:",self.xmin, self.xmax, self.ymin, self.ymax)
        print(filtered_cloud[0])
        # # Convert the filtered point cloud to a ROS message
        fields = [
            pcl2.PointField(name="x", offset=0, datatype=pcl2.PointField.FLOAT32, count=1),
            pcl2.PointField(name="y", offset=4, datatype=pcl2.PointField.FLOAT32, count=1),
            pcl2.PointField(name="z", offset=8, datatype=pcl2.PointField.FLOAT32, count=1),
        ]
        filtered_pc_msg = pcl2.create_cloud(msg.header, fields,filtered_cloud)
        # # Publish the filtered point cloud
        self.filtered_pc_pub.publish(filtered_pc_msg)


if __name__ == '__main__':
    node = PointCloudFilterNode()
    rospy.spin()

