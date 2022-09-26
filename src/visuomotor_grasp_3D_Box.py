import rospy
#import pcl
import os
import time
import random
from std_msgs.msg import Float64, String, Bool
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, SetModelState, SetPhysicsProperties, GetWorldProperties
from gazebo_msgs.msg import ModelState, ODEPhysics, ModelStates
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, Constraints, RobotState, MoveItErrorCodes
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from sensor_msgs.msg import JointState, CompressedImage, PointCloud2, PointField, Image
from sensor_msgs import point_cloud2 as pc2
import cv2
import tf
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

#globals
object_pose = Pose(Point(0,0,0), Quaternion(1,0,0,0))
gripper_open_value = -0.7
PI    = 3.1415
STEPS_OBJECT_POSE = 100
STEPS_CAM_POSE = 10
HOME  = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..'))
DEBUG = True
RGB_IMG_DIR = HOME + "/dataset/RGB_image"
DEPTH_IMG_DIR = HOME + "/dataset/DEPTH_image"
MODEL_DIR = "/home/fujenchu/projects/robotArm/catkin_ws/src/simData/warehouse/models"

### ros service proxies
print("Waiting for gazebo servies, be sure to spawn gazebo_ros with rosrun gazebo_ros gazebo or rosrun gazebo_ros gzserver")
rospy.wait_for_service("/gazebo/spawn_sdf_model")
rospy.wait_for_service("/gazebo/delete_model")
rospy.wait_for_service("/gazebo/get_model_state")
rospy.wait_for_service("/gazebo/set_model_state")
rospy.wait_for_service("/gazebo/set_physics_properties")
rospy.wait_for_service("/gazebo/get_world_properties")

print("Gazebo services are up!")
print("MoiveIt! services are up!")
spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

def main():
    ### start node
    rospy.init_node('gazebo_arm_object')

    world_properties = get_world_properties()
    if DEBUG: print("World properties: "); print(world_properties); print("\n")

    ### load the table
    if "table" not in world_properties.model_names:
        print("Loading the table")
        with open(HOME + "/models/table/model-1_2.sdf", "r") as f:
            table_xml = f.read()
        spawn_model("table", table_xml, "", get_pose(0.50, 0, 0, 0, 0, PI / 2),
                    "world")  # 0.6(height of surface of table)-0.02(half depth of table)

    ###open files for recording ground turth
    f_obj = open(HOME + "/annotation/object_pose.txt", "w")
    f_cam = open(HOME + "/annotation/camera_pose.txt", "w")

    # update camera pose
    Load_Camera(world_properties, None)
    rospy.sleep(0.25)


    for name in sorted(os.listdir(MODEL_DIR)):
        # create objec folder if not exists
        if not os.path.exists(RGB_IMG_DIR + "/" + name):
            os.makedirs(RGB_IMG_DIR + "/" + name)
        if not os.path.exists(DEPTH_IMG_DIR + "/" + name):
            os.makedirs(DEPTH_IMG_DIR + "/" + name)

        count_obj_pose = 0
        while count_obj_pose < STEPS_OBJECT_POSE:
            with open(MODEL_DIR + "/" + name + "/sdf/description.sdf", "r") as f:
                object_xml = f.read()
                pose_obj = rotate_pose_object(count_obj_pose, STEPS_OBJECT_POSE)

            # wait for model to load in
            while "object" not in world_properties.model_names:
                world_properties = get_world_properties()
                print("Model loaded, ready to go")
                spawn_model("object", object_xml, "", pose_obj, "world")
                rospy.sleep(0.25)


            ### take data from kinect
            for x in range(10):
                rgb_data_tmp = rospy.wait_for_message("/camera/depth/image_raw/compressed", CompressedImage, None)
                d_data_tmp = rospy.wait_for_message("/camera/depth/image_raw", Image, None)


            # save rgb images
            try:
                rgb_data = rospy.wait_for_message("/camera/depth/image_raw/compressed", CompressedImage, None)
                # bridge1 = CvBridge()
                np_arr_rgb = np.fromstring(rgb_data.data, np.uint8)
                cv_image_rgb = cv2.imdecode(np_arr_rgb, cv2.IMREAD_COLOR)
                # cv_image_rgb = bridge1.imgmsg_to_cv2(rgb_data, "rgb8")
                print(RGB_IMG_DIR + "/" + name + "/" + name + "_" + str(count_obj_pose) + ".png")
                cv2.imwrite(RGB_IMG_DIR + "/" + name + "/" + name + "_" + str(count_obj_pose) + ".png", cv_image_rgb)
            except CvBridgeError as e:
                print(e)

            # save depth image
            try:
                d_data = rospy.wait_for_message("/camera/depth/image_raw", Image, None)
                # np_arr_d = np.fromstring(d_data.data, np.float32)
                # cv2.imshow('Depth Image', np_arr_d)
                # cv_image_d = cv2.imdecode(np_arr_d, cv2.IMREAD_UNCHANGED)
                bridge = CvBridge()
                cv_image_d = bridge.imgmsg_to_cv2(d_data, "32FC1")
                np_arr_d = np.array(cv_image_d, dtype=np.float32)
                np_arr_d[np.isnan(np_arr_d)] = 0
                cv2.normalize(np_arr_d, np_arr_d, 0, 1, cv2.NORM_MINMAX)
                # cv_image_d = bridge2.imgmsg_to_cv2(d_data, "32FC1")
                print(DEPTH_IMG_DIR + "/" + name + "/" + name +"_" + str(count_obj_pose) +  ".png")
                cv2.imwrite(DEPTH_IMG_DIR + "/" + name + "/" + name +"_" + str(count_obj_pose) +  ".png", np_arr_d * 255)
            except CvBridgeError as e:
                print(e)


            #increment for object pose
            count_obj_pose += 1

            # unload the model
            delete_model("object")
            # wait for model to be deleted
            while "object" in world_properties.model_names:
                world_properties = get_world_properties()
            rospy.sleep(0.25)

    # delete camera model
    delete_model("kinect_ros")
    # wait for camera to be deleted
    while "kinect_ros" in world_properties.model_names:
        world_properties = get_world_properties()

    #close the txt files
    f_obj.close()
    f_cam.close()

def get_pose(x, y, z, R, P, Y):
    tmp_quat = tf.transformations.quaternion_from_euler(R,P,Y)
    quat = Quaternion(tmp_quat[0], tmp_quat[1], tmp_quat[2], tmp_quat[3])
    return Pose(Point(x,y,z), quat)

def rotate_pose_object(count, steps):
    #x = random.uniform(0.2, 0.6)
    #y = random.uniform(-0.3, 0.3)
    steps_srt = math.sqrt(steps)
    step_x = math.floor(count/steps_srt)
    step_y = count%steps_srt
    x = 0.5 - 0.1*0.5 + 0.1*step_x/steps_srt
    y = -0.2 - 0.1*0.5 + 0.1*step_y/steps_srt

    z = 0.6
    #yaw = random.uniform(-1, 1) * PI
    yaw = count * ((2 * PI) / steps)
    tmp_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    quat = Quaternion(tmp_quat[0], tmp_quat[1], tmp_quat[2], tmp_quat[3])
    return Pose(Point(x,y,z), quat)


def rotation_matrix(u1,u2,u3,v1,v2,v3): #unit axis vector of two coordiantes
    #return np.array([[u1.dot(v1),u2.dot(v1),u3.dot(v1)],[u1.dot(v2),u2.dot(v2),u3.dot(v2)],[u1.dot(v3),u2.dot(v3),u3.dot(v3)]])
    #return np.array([[v1.dot(u1), v1.dot(u2), v1.dot(u3)], [v2.dot(u1), v2.dot(u2), v2.dot(u3)],[v3.dot(u1), v3.dot(u2), v3.dot(u3)]])
    return np.array([[u1.dot(v1), u1.dot(v2), u1.dot(v3)], [u2.dot(v1), u2.dot(v2), u2.dot(v3)],
                     [u3.dot(v1), u3.dot(v2), u3.dot(v3)]])

def Load_Camera(world_properties, pose):
    ### load the kinect
    if "kinect_ros" not in world_properties.model_names:
        print("Loading the kinect")
        with open(HOME + "/models/kinect_ros/model.sdf", "r") as f:
            kinect_xml = f.read()
        spawn_model("kinect_ros", kinect_xml, "", None, "world")
        print("Waiting for Kinect topics to publish")
        # just wait for one message to make sure we're good, no timeout hangs forever http://docs.ros.org/diamondback/api/rospy/html/rospy.client-module.html#wait_for_message
        rospy.sleep(1.0)
        rospy.wait_for_message("/camera/depth/image_raw/compressed", CompressedImage,None)
        rospy.wait_for_message("/camera/depth/image_raw", Image, None)

if __name__ == "__main__":
    main()