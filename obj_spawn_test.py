import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import os
import random

NUMBER_OF_OBJECTS = 3

def unspawn_object(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_proxy(model_name)
        rospy.loginfo("Object '%s' has been unspawned.", model_name)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to unspawn object: %s", str(e))

def spawn_object():
    global NUMBER_OF_OBJECTS
    model_name = None
    rospy.init_node('object_spawner', anonymous=True)
    spawn_sdf_model_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    current_file_path = os.path.abspath(__file__)
    current_directory = os.path.dirname(current_file_path)
    while True:
        ans = input("want another spawn:[y/n]")
        # if model_name is not None:
        #     unspawn_object(model_name)
        if ans != "n":
            sdf_path = ""
            object_random = random.randint(0,NUMBER_OF_OBJECTS-1)
            object_pose = Pose()
            object_random = 0
            z_random = random.randint(0,1)
            z_random = 0
            if object_random == 0:
                sdf_path = current_directory+"/bowl_descriptor.sdf"
                if z_random == 0:
                    object_pose.position.z = 0.90
                elif z_random == 1:
                    object_pose.position.z = 0.55
                object_pose.orientation.x = 0.7071068
                object_pose.orientation.y = 0
                object_pose.orientation.z = 0
                object_pose.orientation.w = 0.7071068
                model_name = "bowl"
            elif object_random == 1:
                sdf_path = current_directory+"/bottle_descriptor.sdf"
                if z_random == 0:
                    object_pose.position.z = 0.90
                elif z_random == 1:
                    object_pose.position.z = 0.52
                object_pose.orientation.x = 0
                object_pose.orientation.y = 0
                object_pose.orientation.z = 0
                object_pose.orientation.w = 1
                model_name = "bottle"
            elif object_random == 2:
                sdf_path = current_directory+"/knife_descriptor.sdf"
                object_pose.position.z = 1.4
                object_pose.orientation.x = 0
                object_pose.orientation.y = -0.7071068
                object_pose.orientation.z = 0
                object_pose.orientation.w = 0.7071068
                model_name = "knife"
            else:
                print("this won't happen")
                break
            object_pose.position.x = 0.60
            object_pose.position.y = (random.random()*0.6)-0.3
           

            success = spawn_sdf_model_proxy(model_name, open(sdf_path, 'r').read(), "", object_pose, "world")
            print(model_name,success)
            print(object_pose.position)
        else:
            break

if __name__ == '__main__':
    try:
        spawn_object()
    except rospy.ROSInterruptException:
        pass
