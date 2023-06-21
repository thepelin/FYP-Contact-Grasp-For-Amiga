import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

class DepthImageFilterNode(object):
    def __init__(self):
        rospy.init_node('depth_image_filter_node')

        self.filtered_depth_pub = rospy.Publisher('/filtered_depth/image_raw', Image, queue_size=10)
        self.camera_pub=rospy.Publisher('/filtered_depth/camera_info',CameraInfo,queue_size=10)
        self.camera_info_sub = rospy.Subscriber('/zed2_node/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
        self.depth_sub = rospy.Subscriber('/zed2_node/depth/image_raw', Image, self.depth_image_callback)

    def camera_info_callback(self, msg):
        # Store the received camera info
        print("here")
        self.camera_pub.publish(msg)

    def bbox_callback(self, msg):
        # Store the received bounding box coordinates
        print("bbox_callback")
        found = False
        for bb in msg.bounding_boxes:
            if bb.Class == "bottle" or  bb.Class == "vase":
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


    def depth_image_callback(self, msg):
        # Convert the received depth image message to a NumPy array
        print("depth_callback")
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_image = np.array(depth_image, dtype=np.float32)

        # Get the bounding box coordinates
        bbox_xmin = self.xmin
        bbox_xmax = self.xmax
        bbox_ymin = self.ymin
        bbox_ymax = self.ymax

        # Filter the depth image based on the bounding box
        filtered_depth_image = depth_image.copy()
        filtered_depth_image[filtered_depth_image < bbox_xmin] = np.nan
        filtered_depth_image[filtered_depth_image > bbox_xmax] = np.nan
        filtered_depth_image[:, :bbox_ymin] = np.nan
        filtered_depth_image[:, bbox_ymax:] = np.nan
        
        # Convert the filtered depth image to an Image message
        filtered_depth_image_msg = bridge.cv2_to_imgmsg(filtered_depth_image, encoding="passthrough")
        filtered_depth_image_msg.header = msg.header
        # Publish the filtered depth image
        self.filtered_depth_pub.publish(filtered_depth_image_msg)

if __name__ == '__main__':
    node = DepthImageFilterNode()
    rospy.spin()
