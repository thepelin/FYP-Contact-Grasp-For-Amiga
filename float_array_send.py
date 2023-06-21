import rospy
from std_msgs.msg import Float64MultiArray

while True:
    rospy.init_node('array_publisher', anonymous=True)

    # Create a publisher for the Float64MultiArray message
    array_publisher = rospy.Publisher('/transformation_matrix', Float64MultiArray, queue_size=10)

    # Create a Float64MultiArray message
    array_msg = Float64MultiArray()

    # Set the data of the array
    array_msg.data = [ 0.8762379 ,  0.0306624 , -0.48090222,  0.10491548,
                    -0.05524716,  0.9977853 , -0.03704529,  0.20511378,
                    0.4787012 ,  0.05902897,  0.8759913 ,  0.5174361,
                        0.        ,  0.        ,  0.        ,  1.        ]

    # Publish the array message
    array_publisher.publish(array_msg)