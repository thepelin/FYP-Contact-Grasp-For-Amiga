import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

def transform_callback(ros_point_cloud):
    # Apply the transform
    transformed_cloud = do_transform_cloud(ros_point_cloud, transform_stamped)

    # Publish the transformed point cloud
    pub.publish(transformed_cloud)

if __name__ == "__main__":
    rospy.init_node('point_cloud_transformer')

    origin_frame = "origin_frame"  # Replace with the actual origin frame name
    goal_frame = "goal_frame"  # Replace with the actual goal frame name

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Look up the transform between goal_frame and origin_frame
    try:
        transform_stamped = tf_buffer.lookup_transform(goal_frame, origin_frame, rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to lookup transform between {} and {}".format(goal_frame, origin_frame))
        exit(1)

    # Set up the point cloud subscriber and publisher
    rospy.Subscriber('/your/point_cloud_topic', PointCloud2, transform_callback)
    pub = rospy.Publisher('/transformed_point_cloud_topic', PointCloud2, queue_size=10)

    rospy.spin()