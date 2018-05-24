#!/usr/bin/python

#Kinect camera sensor coordinate
#############################
#        |  Y               #
#        |                  #
#        |       X          #
#        |--------          #
#      |                    #
#    |                      #
#  | Z-sensor direction     #
#############################

#ee bbx definition (counter-closewise)
########################
#     4*-------3*      #
#   1*|------2*|       #
#   | |      | |       #
#   | |      | |       #
#   | 8*-----|-7*      #
#   5*-------6*        #
########################

import rospy
import pcl
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
STEPS_ARM_POSE = 100
STEPS_OBEJCT_NUM = 90
STEPS_OBJECT_RANDOM_POSE = 1
STEPS_OBJECT_CLOSE_POSE = 1
STEPS_CAM_POSE = 1
HOME  = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..'))
DEBUG = True
MODEL_NAME = ("box", "cylinder", "sphere")
RGB_IMG_DIR = HOME + "/dataset/RGB_image"
DEPTH_IMG_DIR = HOME + "/dataset/DEPTH_image"

### ros service proxies
print("Waiting for gazebo servies, be sure to spawn gazebo_ros with rosrun gazebo_ros gazebo or rosrun gazebo_ros gzserver")
rospy.wait_for_service("/gazebo/spawn_sdf_model")
rospy.wait_for_service("/gazebo/delete_model")
rospy.wait_for_service("/gazebo/get_model_state")
rospy.wait_for_service("/gazebo/set_model_state")
rospy.wait_for_service("/gazebo/set_physics_properties")
rospy.wait_for_service("/gazebo/get_world_properties")
print("Waiting for ROS Moveit! servies")
rospy.wait_for_service("/compute_ik")

print("Gazebo services are up!")
print("MoiveIt! services are up!")
spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
get_position_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

def main():
    ### start node
    rospy.init_node('gazebo_arm_obejct')

    #initialize publishers for 9 motors
    pub_1 = rospy.Publisher('/finalarm_released_v0/joint_1_position_controller/command', Float64, queue_size=5)
    pub_2 = rospy.Publisher('/finalarm_released_v0/joint_2_position_controller/command', Float64, queue_size=5)
    pub_3 = rospy.Publisher('/finalarm_released_v0/joint_3_position_controller/command', Float64, queue_size=5)
    pub_4 = rospy.Publisher('/finalarm_released_v0/joint_4_position_controller/command', Float64, queue_size=5)
    pub_5 = rospy.Publisher('/finalarm_released_v0/joint_5_position_controller/command', Float64, queue_size=5)
    pub_6 = rospy.Publisher('/finalarm_released_v0/joint_6_position_controller/command', Float64, queue_size=5)
    pub_7 = rospy.Publisher('/finalarm_released_v0/joint_7_position_controller/command', Float64, queue_size=5)
    pub_8 = rospy.Publisher('/finalarm_released_v0/joint_8_position_controller/command', Float64, queue_size=5)
    pub_9 = rospy.Publisher('/finalarm_released_v0/joint_9_position_controller/command', Float64, queue_size=5)

    ### disable physics
    # TODO change this to just disabling Kinect physics so the model can drop onto the table
    # physics = set_physics(None, None, None, ODEPhysics(None, None, 0, None, None, None, None, None, None, None))
    # if DEBUG: print("Updated physics: "); print(physics); print("\n")

    world_properties = get_world_properties()
    if DEBUG: print("World properties: "); print(world_properties); print("\n")

    ### load the table
    if "table" not in world_properties.model_names:
        print("Loading the table")
        with open(HOME + "/models/table/model-1_2.sdf", "r") as f:
                table_xml = f.read()
        spawn_model("table", table_xml, "", get_pose(0.50, 0, 0, 0, 0, PI/2), "world")  #0.6(height of surface of table)-0.02(half depth of table)

    ###variables
    count_arm_pose = 0

    ###open files for recording ground turth
    f_arm = open(HOME + "/annotation/end_effector_pose.txt", "a", 0)
    f_obj = open(HOME + "/annotation/object_pose.txt", "a", 0)
    f_cam = open(HOME + "/annotation/camera_pose.txt", "a", 0)
    f_bbx_ee = open(HOME + "/annotation/bbx_end_effector.txt", "a", 0)
    f_bbx_obj = open(HOME + "/annotation/bbx_object.txt", "a", 0)

    ###loop for arm pose
    while count_arm_pose < STEPS_ARM_POSE:
        req = PositionIKRequest()
        req.group_name = "arm"
        req.pose_stamped.header.frame_id = ""
        req.avoid_collisions = True
        valid_pose = 0

        ###generate random pose and compute ik
        while not valid_pose:
            req.pose_stamped.pose, rotate_angle = random_pose_arm()
            #req.pose_stamped.pose = get_pose(0.3, 0.2, -0.12, 0, PI,PI + PI/4)
            #req.pose_stamped.pose = Pose(Point(0.3,0.2,-0.12),Quaternion(1, 0, 0, 0))
            rospy.wait_for_service("/compute_ik", 1.0)
            ikrep = get_position_ik(req)
            if ikrep.error_code.val == ikrep.error_code.SUCCESS:
                valid_pose = 1

        ###execute the computed joint values
        rospy.sleep(1.0)
        execution_pose(pub_1, pub_2, pub_3, pub_4, pub_5, pub_6, pub_7, pub_8, pub_9, ikrep)

        ###################################
        # loop for different object model #
        ###################################
        count_object_num = 0
        while count_object_num < STEPS_OBEJCT_NUM:
            ### loop for diff random object pose
            count_obejct_random_pose = 0
            count_object_close_pose = 0
            while count_obejct_random_pose < STEPS_OBJECT_RANDOM_POSE:
                #generate new pose for object
                pose_obj = random_pose_object(req.pose_stamped.pose.position.x, req.pose_stamped.pose.position.y)
                #pose_obj = Pose(Point(0.40,0.2,0.63),Quaternion(1, 0, 0, 0))
                ### Build the model
                print("Loading the SDF from " + MODEL_NAME[count_object_num])
                # load sdf xml
                with open(HOME + "/models/" + MODEL_NAME[count_object_num] + "/description.sdf", "r") as f:
                    model_xml = f.read()
                # call spawn sdf model gazebo service with intial pose
                spawn_model("object", model_xml, "", pose_obj, "world")

                # wait for model to load in
                while "object" not in world_properties.model_names:
                    world_properties = get_world_properties()
                    print("Model loaded, ready to go")
                #model_state = ModelState("object", pose_obj, None, "world")
                #set_model_state(model_state)
                rospy.sleep(1.0)  # wait for model to become static since model may be collided with arm

                ###loop for diff camera pose
                count_cam_pose = 0
                while count_cam_pose < STEPS_CAM_POSE:
                    ###update camera pose
                    pose_cam = random_pose_cam(pose_obj.position.x, pose_obj.position.y, pose_obj.position.z)

                    #spawn kinect camera with generated pose
                    Load_Camera(world_properties, pose_cam)

                    ### take data from kinect
                    rgb_data = rospy.wait_for_message("/camera/depth/image_raw/compressed", CompressedImage, None)
                    d_data = rospy.wait_for_message("/camera/depth/image_raw", Image, None)
                    #point_data = rospy.wait_for_message("/camera/depth/points", PointCloud2, None)

                    #get the time stamp
                    ts = int(time.time())

                    #save rgb images
                    try:
                        #bridge1 = CvBridge()
                        np_arr_rgb = np.fromstring(rgb_data.data, np.uint8)
                        cv_image_rgb = cv2.imdecode(np_arr_rgb, cv2.IMREAD_COLOR)
                        #cv_image_rgb = bridge1.imgmsg_to_cv2(rgb_data, "rgb8")
                        print(RGB_IMG_DIR + "/" + str(ts) + ".png")
                        cv2.imwrite(RGB_IMG_DIR + "/" + str(ts) + ".png", cv_image_rgb)
                    except CvBridgeError as e:
                        print(e)

                    #save depth image
                    try:
                        #np_arr_d = np.fromstring(d_data.data, np.float32)
                        #cv2.imshow('Depth Image', np_arr_d)
                        #cv_image_d = cv2.imdecode(np_arr_d, cv2.IMREAD_UNCHANGED)
                        bridge = CvBridge()
                        cv_image_d = bridge.imgmsg_to_cv2(d_data, "32FC1")
                        np_arr_d = np.array(cv_image_d, dtype=np.float32)
                        np_arr_d[np.isnan(np_arr_d)] = 0
                        cv2.normalize(np_arr_d, np_arr_d, 0, 1, cv2.NORM_MINMAX)
                        #cv_image_d = bridge2.imgmsg_to_cv2(d_data, "32FC1")
                        print(DEPTH_IMG_DIR + "/" + str(ts) + ".png")
                        cv2.imwrite(DEPTH_IMG_DIR + "/" + str(ts) + ".png", np_arr_d*255)
                    except CvBridgeError as e:
                        print(e)

                    # save point cloud
                    # points_list = []
                    # for data in pc2.read_points(point_data, skip_nans = True):
                    #    points_list.append([data[0], data[1], data[2], data[3]])
                    # pcl_data = pcl.PointCloud_PointXYZRGB()
                    # pcl_data.from_list(points_list)
                    # pcl.save(DEPTH_IMG_DIR + "/" + str(ts) + ".pcd", pcl_data)
                    # pcl_data.to_file(DEPTH_IMG_DIR + "/" + str(ts) + ".pcd", ascii = True)

                    #delete camera model
                    delete_model("kinect_ros")

                    ###wait the kinect model to be deleted
                    while "kinect_ros" in world_properties.model_names:
                        world_properties = get_world_properties()

                    ###save ground truth
                    #arm pose
                    f_arm.write(str(ts) + " " + str(req.pose_stamped.pose.position.x) + " " + str(
                        req.pose_stamped.pose.position.y) + " "
                                + str(req.pose_stamped.pose.position.z) + " " + str(
                        req.pose_stamped.pose.orientation.x) + " "
                                + str(req.pose_stamped.pose.orientation.y) + " " + str(
                        req.pose_stamped.pose.orientation.z) + " "
                                + str(req.pose_stamped.pose.orientation.w) + "\n")
                    #object pose
                    f_obj.write(str(ts) + " " + MODEL_NAME[count_object_num] + str(object_pose.position.x) + " "
                                + str(object_pose.position.y) + " " + str(object_pose.position.z) + " "
                                + str(object_pose.orientation.x) + " " + str(object_pose.orientation.y) + " "
                                + str(object_pose.orientation.z) + " " + str(object_pose.orientation.w) + "\n")
                    #cam pose
                    f_cam.write(str(ts) + " " + str(pose_cam.position.x) + " "
                                + str(pose_cam.position.y) + " " + str(pose_cam.position.z) + " "
                                + str(pose_cam.orientation.x) + " " + str(pose_cam.orientation.y) + " "
                                + str(pose_cam.orientation.z) + " " + str(pose_cam.orientation.w) + "\n")
                    #bbx_ee
                    #x y z ...... x y z
                    x_incre_1 = 0.05 * np.sqrt(2) * np.sin((45 - rotate_angle) / 180 * PI)
                    x_incre_2 = 0.05 * np.sqrt(2) * np.sin((45 + rotate_angle) / 180 * PI)
                    x_incre_3 = 0.05 * np.sqrt(2) * np.sin((45 - rotate_angle) / 180 * PI)
                    x_incre_4 = 0.05 * np.sqrt(2) * np.sin((45 + rotate_angle) / 180 * PI)
                    y_incre_1 = 0.05 * np.sqrt(2) * np.cos((45 - rotate_angle) / 180 * PI)
                    y_incre_2 = 0.05 * np.sqrt(2) * np.cos((45 + rotate_angle) / 180 * PI)
                    y_incre_3 = 0.05 * np.sqrt(2) * np.cos((45 - rotate_angle) / 180 * PI)
                    y_incre_4 = 0.05 * np.sqrt(2) * np.cos((45 + rotate_angle) / 180 * PI)

                    f_bbx_ee.write(str(ts) + " " + str(req.pose_stamped.pose.position.x - x_incre_1) + " "
                                   + str(req.pose_stamped.pose.position.y + y_incre_1) + " " + str(req.pose_stamped.pose.position.z + 0.05) + " "
                                   + str(req.pose_stamped.pose.position.x + x_incre_2) + " " + str(req.pose_stamped.pose.position.y + y_incre_2) + " "
                                   + str(req.pose_stamped.pose.position.z + 0.05) + " " + str(req.pose_stamped.pose.position.x + x_incre_3) + " "
                                   + str(req.pose_stamped.pose.position.y - y_incre_3) + " " + str(req.pose_stamped.pose.position.z + 0.05) + " "
                                   + str(req.pose_stamped.pose.position.x - x_incre_4) + " " + str(req.pose_stamped.pose.position.y - y_incre_4) + " "
                                   + str(req.pose_stamped.pose.position.z + 0.05) + " "
                                   + str(req.pose_stamped.pose.position.x - x_incre_1) + " "
                                   + str(req.pose_stamped.pose.position.y + y_incre_1) + " " + str(req.pose_stamped.pose.position.z - 0.07) + " "
                                   + str(req.pose_stamped.pose.position.x + x_incre_2) + " " + str(req.pose_stamped.pose.position.y + y_incre_2) + " "
                                   + str(req.pose_stamped.pose.position.z - 0.07) + " " + str(req.pose_stamped.pose.position.x + x_incre_3) + " "
                                   + str(req.pose_stamped.pose.position.y - y_incre_3) + " " + str(req.pose_stamped.pose.position.z - 0.07) + " "
                                   + str(req.pose_stamped.pose.position.x - x_incre_4) + " " + str(req.pose_stamped.pose.position.y - y_incre_4) + " "
                                   + str(req.pose_stamped.pose.position.z - 0.07) + "\n")

                    #increment count for cam pose
                    count_cam_pose += 1

                ###increment count for obj pose
                count_obejct_random_pose += 1

            ###################################
            # loop for diff close object pose #
            ###################################
            while count_object_close_pose < STEPS_OBJECT_CLOSE_POSE:
                #generate new pose for object
                pose_obj = random_pose_object_close(req.pose_stamped.pose.position.x, req.pose_stamped.pose.position.y)
                #pose_obj = Pose(Point(0.40,0.2,0.63),Quaternion(1, 0, 0, 0))
                ### Build the model
                print("Loading the SDF from " + MODEL_NAME[count_object_num])
                # load sdf xml
                with open(HOME + "/models/" + MODEL_NAME[count_object_num] + "/description.sdf", "r") as f:
                    model_xml = f.read()
                # call spawn sdf model gazebo service with intial pose
                spawn_model("object", model_xml, "", pose_obj, "world")

                # wait for model to load in
                while "object" not in world_properties.model_names:
                    world_properties = get_world_properties()
                    print("Model loaded, ready to go")
                #model_state = ModelState("object", pose_obj, None, "world")
                #set_model_state(model_state)
                rospy.sleep(1.0)  # wait for model to become static since model may be collided with arm

                ###loop for diff camera pose
                count_cam_pose = 0
                while count_cam_pose < STEPS_CAM_POSE:
                    ###update camera pose
                    pose_cam = random_pose_cam(pose_obj.position.x, pose_obj.position.y, pose_obj.position.z)

                    #spawn kinect camera with generated pose
                    Load_Camera(world_properties, pose_cam)

                    ### take data from kinect
                    rgb_data = rospy.wait_for_message("/camera/depth/image_raw/compressed", CompressedImage, None)
                    d_data = rospy.wait_for_message("/camera/depth/image_raw", Image, None)
                    #point_data = rospy.wait_for_message("/camera/depth/points", PointCloud2, None)

                    #get the time stamp
                    ts = int(time.time())

                    #save rgb images
                    try:
                        #bridge1 = CvBridge()
                        np_arr_rgb = np.fromstring(rgb_data.data, np.uint8)
                        cv_image_rgb = cv2.imdecode(np_arr_rgb, cv2.IMREAD_COLOR)
                        #cv_image_rgb = bridge1.imgmsg_to_cv2(rgb_data, "rgb8")
                        print(RGB_IMG_DIR + "/" + str(ts) + ".png")
                        cv2.imwrite(RGB_IMG_DIR + "/" + str(ts) + ".png", cv_image_rgb)
                    except CvBridgeError as e:
                        print(e)

                    #save depth image
                    try:
                        #np_arr_d = np.fromstring(d_data.data, np.float32)
                        #cv2.imshow('Depth Image', np_arr_d)
                        #cv_image_d = cv2.imdecode(np_arr_d, cv2.IMREAD_UNCHANGED)
                        bridge = CvBridge()
                        cv_image_d = bridge.imgmsg_to_cv2(d_data, "32FC1")
                        np_arr_d = np.array(cv_image_d, dtype=np.float32)
                        np_arr_d[np.isnan(np_arr_d)] = 0
                        cv2.normalize(np_arr_d, np_arr_d, 0, 1, cv2.NORM_MINMAX)
                        #cv_image_d = bridge2.imgmsg_to_cv2(d_data, "32FC1")
                        print(DEPTH_IMG_DIR + "/" + str(ts) + ".png")
                        cv2.imwrite(DEPTH_IMG_DIR + "/" + str(ts) + ".png", np_arr_d*255)
                    except CvBridgeError as e:
                        print(e)

                    # save point cloud
                    # points_list = []
                    # for data in pc2.read_points(point_data, skip_nans = True):
                    #    points_list.append([data[0], data[1], data[2], data[3]])
                    # pcl_data = pcl.PointCloud_PointXYZRGB()
                    # pcl_data.from_list(points_list)
                    # pcl.save(DEPTH_IMG_DIR + "/" + str(ts) + ".pcd", pcl_data)
                    # pcl_data.to_file(DEPTH_IMG_DIR + "/" + str(ts) + ".pcd", ascii = True)

                    #delete camera model
                    delete_model("kinect_ros")

                    ###wait the kinect model to be deleted
                    while "kinect_ros" in world_properties.model_names:
                        world_properties = get_world_properties()

                    ###save ground truth
                    #arm pose
                    f_arm.write(str(ts) + " " + str(req.pose_stamped.pose.position.x) + " " + str(
                        req.pose_stamped.pose.position.y) + " "
                                + str(req.pose_stamped.pose.position.z) + " " + str(
                        req.pose_stamped.pose.orientation.x) + " "
                                + str(req.pose_stamped.pose.orientation.y) + " " + str(
                        req.pose_stamped.pose.orientation.z) + " "
                                + str(req.pose_stamped.pose.orientation.w) + "\n")
                    #object pose
                    f_obj.write(str(ts) + " " + MODEL_NAME[count_object_num] + " " + str(object_pose.position.x) + " "
                                + str(object_pose.position.y) + " " + str(object_pose.position.z) + " "
                                + str(object_pose.orientation.x) + " " + str(object_pose.orientation.y) + " "
                                + str(object_pose.orientation.z) + " " + str(object_pose.orientation.w) + "\n")
                    #cam pose
                    f_cam.write(str(ts) + " " + str(pose_cam.position.x) + " "
                                + str(pose_cam.position.y) + " " + str(pose_cam.position.z) + " "
                                + str(pose_cam.orientation.x) + " " + str(pose_cam.orientation.y) + " "
                                + str(pose_cam.orientation.z) + " " + str(pose_cam.orientation.w) + "\n")
                    #bbx_ee
                    #x y z ...... x y z
                    x_incre_1 = 0.05 * np.sqrt(2) * np.sin((45 - rotate_angle) / 180 * PI)
                    x_incre_2 = 0.05 * np.sqrt(2) * np.sin((45 + rotate_angle) / 180 * PI)
                    x_incre_3 = 0.05 * np.sqrt(2) * np.sin((45 - rotate_angle) / 180 * PI)
                    x_incre_4 = 0.05 * np.sqrt(2) * np.sin((45 + rotate_angle) / 180 * PI)
                    y_incre_1 = 0.05 * np.sqrt(2) * np.cos((45 - rotate_angle) / 180 * PI)
                    y_incre_2 = 0.05 * np.sqrt(2) * np.cos((45 + rotate_angle) / 180 * PI)
                    y_incre_3 = 0.05 * np.sqrt(2) * np.cos((45 - rotate_angle) / 180 * PI)
                    y_incre_4 = 0.05 * np.sqrt(2) * np.cos((45 + rotate_angle) / 180 * PI)

                    f_bbx_ee.write(str(ts) + " " + str(req.pose_stamped.pose.position.x - x_incre_1) + " "
                                   + str(req.pose_stamped.pose.position.y + y_incre_1) + " " + str(req.pose_stamped.pose.position.z + 0.05) + " "
                                   + str(req.pose_stamped.pose.position.x + x_incre_2) + " " + str(req.pose_stamped.pose.position.y + y_incre_2) + " "
                                   + str(req.pose_stamped.pose.position.z + 0.05) + " " + str(req.pose_stamped.pose.position.x + x_incre_3) + " "
                                   + str(req.pose_stamped.pose.position.y - y_incre_3) + " " + str(req.pose_stamped.pose.position.z + 0.05) + " "
                                   + str(req.pose_stamped.pose.position.x - x_incre_4) + " " + str(req.pose_stamped.pose.position.y - y_incre_4) + " "
                                   + str(req.pose_stamped.pose.position.z + 0.05) + " "
                                   + str(req.pose_stamped.pose.position.x - x_incre_1) + " "
                                   + str(req.pose_stamped.pose.position.y + y_incre_1) + " " + str(req.pose_stamped.pose.position.z - 0.07) + " "
                                   + str(req.pose_stamped.pose.position.x + x_incre_2) + " " + str(req.pose_stamped.pose.position.y + y_incre_2) + " "
                                   + str(req.pose_stamped.pose.position.z - 0.07) + " " + str(req.pose_stamped.pose.position.x + x_incre_3) + " "
                                   + str(req.pose_stamped.pose.position.y - y_incre_3) + " " + str(req.pose_stamped.pose.position.z - 0.07) + " "
                                   + str(req.pose_stamped.pose.position.x - x_incre_4) + " " + str(req.pose_stamped.pose.position.y - y_incre_4) + " "
                                   + str(req.pose_stamped.pose.position.z - 0.07) + "\n")

                    #increment count for cam pose
                    count_cam_pose += 1

                ###increment count for obj pose
                count_object_close_pose += 1


            ###delete the object model
            delete_model("object")

            ###wait the object model to be deleted
            while "obejct" in world_properties.model_names:
                world_properties = get_world_properties()

            ###increment count for obj no
            count_object_num += 1

        ###increment count for arm pose
        count_arm_pose += 1

    #close all open files
    f_arm.close()
    f_obj.close()
    f_cam.close()
    f_bbx_ee.close()
    f_bbx_obj.close()

def get_pose(x, y, z, R, P, Y):
    tmp_quat = tf.transformations.quaternion_from_euler(R,P,Y)
    quat = Quaternion(tmp_quat[0], tmp_quat[1], tmp_quat[2], tmp_quat[3])
    return Pose(Point(x,y,z), quat) # create pose type

def random_pose_object(x_arm,y_arm):
    x = random.uniform(-0.3, 0.3)
    y = random.uniform(-0.3, 0.3)
    x = x_arm + x;
    y = y_arm + y;
    z = 0.6
    yaw = random.uniform(-1, 1) * PI
    tmp_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    quat = Quaternion(tmp_quat[0], tmp_quat[1], tmp_quat[2], tmp_quat[3])
    return Pose(Point(x,y,z), quat)

def random_pose_object_close(x_arm,y_arm):
    x = random.uniform(-0.02, 0.02)
    y = random.uniform(-0.02, 0.02)
    z = 0.6
    yaw = random.randint(-30, 30) / 180 * PI
    tmp_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    quat = Quaternion(tmp_quat[0], tmp_quat[1], tmp_quat[2], tmp_quat[3])
    return Pose(Point(x,y,z), quat)

def random_pose_arm():   #generate random pose within a region with fixed quaternion(1,0,0,0): x:(0,0.6) y:(-0.3,0.3) z:(-0.2,0.2)
    x = random.uniform(0.15, 0.50)
    y = random.uniform(-0.3, 0.3)
    z = random.uniform(-0.15, 0.2)
    #quat = Quaternion(1, 0, 0, 0)
    Y_angle = random.randint(-90,90)
    Y =  Y_angle / 180 * PI
    tmp_quat = tf.transformations.quaternion_from_euler(0, PI, PI + Y)
    quat = Quaternion(tmp_quat[0], tmp_quat[1], tmp_quat[2], tmp_quat[3])
    return Pose(Point(x,y,z), quat),Y_angle

def random_pose_cam(x_obj,y_obj,z_obj):  #position of object to help generate valid camera pose
    x = random.uniform(-0.35, -0.05)
    ###camera always from left side of arm
    y = random.uniform(0.3, 0.5)
    ###camera from left side or right side
    #if random.uniform(0,1) > 0.5:
    #    y = random.uniform(0.3, 0.5)
    #else:
    #    y = random.uniform(-0.5, -0.3)
    z = random.uniform(1.15, 1.35)
    u1 = np.array([1,0,0])
    u2 = np.array([0,1,0])
    u3 = np.array([0,0,1])
    v1 = np.array([x_obj - x, y_obj - y, z_obj - z]) / np.linalg.norm(([x_obj - x, y_obj - y, z_obj - z]))
    v2 = np.cross(u3,v1)
    if v2[0] < 0:
        v2 = np.cross(v1,u3)
    v2 = v2 / np.linalg.norm(v2)
    v3 = np.cross(v1,v2)
    rot_mat = rotation_matrix(u1,u2,u3,v1,v2,v3)
    M = np.identity(4)
    M[:3, :3] = rot_mat
    tmp_quat = tf.transformations.quaternion_from_matrix(M)
    quat = Quaternion(tmp_quat[0], tmp_quat[1], tmp_quat[2], tmp_quat[3])
    return Pose(Point(x,y,z),quat)


def rotation_matrix(u1,u2,u3,v1,v2,v3): #unit axis vector of two coordiantes
    #return np.array([[u1.dot(v1),u2.dot(v1),u3.dot(v1)],[u1.dot(v2),u2.dot(v2),u3.dot(v2)],[u1.dot(v3),u2.dot(v3),u3.dot(v3)]])
    #return np.array([[v1.dot(u1), v1.dot(u2), v1.dot(u3)], [v2.dot(u1), v2.dot(u2), v2.dot(u3)],[v3.dot(u1), v3.dot(u2), v3.dot(u3)]])
    return np.array([[u1.dot(v1), u1.dot(v2), u1.dot(v3)], [u2.dot(v1), u2.dot(v2), u2.dot(v3)],
                     [u3.dot(v1), u3.dot(v2), u3.dot(v3)]])

def execution_pose(pub_1, pub_2, pub_3 ,pub_4, pub_5, pub_6, pub_7, pub_8, pub_9, ikrep):
    pub_1.publish(ikrep.solution.joint_state.position[0])
    pub_2.publish(ikrep.solution.joint_state.position[1])
    pub_3.publish(ikrep.solution.joint_state.position[2])
    pub_4.publish(ikrep.solution.joint_state.position[3])
    pub_5.publish(ikrep.solution.joint_state.position[4])
    pub_6.publish(ikrep.solution.joint_state.position[5])
    pub_7.publish(ikrep.solution.joint_state.position[6])
    pub_8.publish(gripper_open_value)
    pub_9.publish(-gripper_open_value)
    rospy.sleep(1.0)

def Load_Camera(world_properties, pose):
    ### load the kinect
    if "kinect_ros" not in world_properties.model_names:
        print("Loading the kinect")
        with open(HOME + "/models/kinect_ros/model.sdf", "r") as f:
            kinect_xml = f.read()
        spawn_model("kinect_ros", kinect_xml, "", pose, "world")
        print("Waiting for Kinect topics to publish")
        # just wait for one message to make sure we're good, no timeout hangs forever http://docs.ros.org/diamondback/api/rospy/html/rospy.client-module.html#wait_for_message
        rospy.wait_for_message("/camera/depth/image_raw/compressed", CompressedImage,None)
        rospy.wait_for_message("/camera/depth/points", PointCloud2, None)

if __name__ == "__main__":
    main()


