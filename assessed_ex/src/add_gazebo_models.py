#!/usr/bin/env python

import rospy
import rospkg
import sys

# Baxter SDK that provides high-level functions to control Baxter
import baxter_interface
from baxter_interface import CHECK_VERSION

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    Pose,
    Point,
)
from std_msgs.msg import (
    Empty,
)

noOfBlocks = 5

def load_gazebo_models():

    table1_pose=Pose(position=Point(x=0.8, y=0.5, z=0.0))
    table2_pose=Pose(position=Point(x=-0.2, y=1.0, z=0.0))
    table_reference_frame="world"
    block_reference_frame="world"

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')


    # Spawn First Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table_1", table_xml, "/",
                             table1_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Second Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table_2", table_xml, "/",
                             table2_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Blocks!
    import random
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    for i in range(0,noOfBlocks):
        pos_x = random.uniform(0.4, 0.6)
        pos_y = random.uniform(0.3,0.8)
        #block_pose=Pose(position=Point(x=0.6725, y=0.1265, z=0.7825))
        block_pose=Pose(position=Point(x=pos_x, y=pos_y, z=0.7825))
	rospy.logwarn(block_pose)

        # Spawn Block URDF
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("block"+str(i), block_xml, "/",
                                   block_pose, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table_1")
        resp_delete = delete_model("cafe_table_2")
        for i in range(0,noOfBlocks):
            resp_delete = delete_model("block"+str(i))
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def move_neutral_position():
    _left_arm = baxter_interface.Limb("left")
    _right_arm = baxter_interface.Limb("right")
    _left_arm.move_to_neutral()
    _right_arm.move_to_neutral()
    _left_arm.exit_control_mode()
    _right_arm.exit_control_mode()
    rospy.loginfo("Enabling Baxter completed!")

def main():
    rospy.init_node("add_gazebo_models")
    
    # Setup Baxter SDK
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rospy.loginfo("Enabling Baxter")
    # You need to enable Baxter to use it!
    rs.enable()
    # Move to home position
    move_neutral_position()

    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)
    
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
