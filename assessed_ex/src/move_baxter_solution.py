#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import moveit_commander
import tf2_ros
from transforms3d import quaternions
import numpy as np  

noOfBlocks = 5

class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        
        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        #We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")
        self._group.set_max_velocity_scaling_factor(0.2) # This is to make Baxter move slower


    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")


    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)


    def _servo_to_pose(self, pose):
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)
        

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def main():
    #Initialise move it interface
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node("ik_pick_and_place_moveit")
    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)

    #Define where the starting pose will be
    starting_pose = Pose(
        position=Point(x=0.7, y=0.15, z=1.2),
        orientation=overhead_orientation)
    pnp = PickAndPlaceMoveIt(limb, hover_distance)
    #Move baxter to the start
    pnp.move_to_start(starting_pose)
    
    idx = 0
    #Initialise the tf listener which recieves the blocks positions from the gaxebot2fframe broadcast
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    #The middle of the table's x coordinate
    start=-0.2
    rate=rospy.Rate(1.0)
    #loop for the number of blocks 
    for idx in range(0, noOfBlocks):
        current_block = "block"+str(idx)
        try:
            #Get the transformation
            transformation = tfBuffer.lookup_transform('base',current_block,rospy.Time(), rospy.Duration(1.0))
            #Get the translation transformation and extract the x, y and z coordinate values
            trans = transformation.transform.translation
            x = trans.x
            y = trans.y
            z = trans.z

            #Define where the current block pose will be
            current_pose = (Pose(
            position=Point(x, y, z),
            orientation=overhead_orientation))
            print("\nPicking...")
            #Start the pick movement with the block coordinates
            pnp.pick(current_pose)
            print("\nPlacing...")
            #Define where the place pose will be
            place_pose = (Pose(
            position=Point(x =start, y = 1.0, z =0.8),
            orientation=overhead_orientation))
            #Start the place movement
            pnp.place(place_pose)
            start = start +0.06
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            
    return 0

if __name__ == '__main__':
    sys.exit(main())



