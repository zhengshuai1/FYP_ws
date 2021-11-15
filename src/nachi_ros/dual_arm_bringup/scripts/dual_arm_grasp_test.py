#!/usr/bin/env python
from __future__ import print_function


import rospy
from roport.srv import *
import numpy as np
from utils import ros_utils
from utils.dual_arm_control import DualArmCommander
from utils.transform import Rotation, Transform

class NachiGraspController(object):
    """Nachi robot grasp Control"""

    def __init__(self):
        super(NachiGraspController, self).__init__()
        # self.tf_tree = ros_utils.TransformTree()
        self.mc = DualArmCommander()
        rospy.loginfo("Ready to take action")

    def plan_trajectories(self, group_name, grasp, cls):
        rospy.loginfo("execute plan trajectories")
        T_base_grasp = grasp
        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_base_pregrasp = T_grasp_pregrasp * T_base_grasp
        T_base_retreat = T_grasp_retreat * T_base_grasp
        T_base_middle, T_base_place = self.select_place_pose(T_base_grasp.rotation, group_name, cls)

        # T_place_preplace = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        # # T_base_preplace = T_place_preplace * T_base_place

        trajectory1 = [T_base_middle, T_base_pregrasp, T_base_grasp]
        trajectory2 = [T_base_retreat, T_base_middle, T_base_place]
        rospy.loginfo("finish plan trajectories")
        return trajectory1, trajectory2

    def select_place_pose(self, rotation, group_name, cls):
        translation = [0.161256, 0.279421, 0.165748]
        middle_translation = [0.161256, 0.279421, 0.25]
        if cls == 'wire':
            translation = translation
        elif cls == 'pcb':
            translation[0] = translation[0] - 0.1
        place_rotation = Rotation.from_quat([1, 0, 0, 0])
        T_base_place = Transform(place_rotation, translation)

        if group_name == 'right_arm':
            T_base_middle =Transform(place_rotation, middle_translation)
            return T_base_middle, T_base_place
        elif group_name == 'left_arm':
            T_base_middle = Transform(Rotation.from_quat([0, 1, 0, 0]),
                                      [middle_translation[0], -middle_translation[1], middle_translation[2]])
            T_base_place.translation[1] = -T_base_place.translation[1]  # -y left arm
            T_base_place.rotation = Rotation.from_quat([0, 1, 0, 0])
            return T_base_middle, T_base_place
        else:
            rospy.logerr('Unknown group name')

    def execute_trajectory(self, group_name, trajectory):
        rospy.loginfo("execute trajectory")
        self.mc.goto_many_poses(group_name, trajectory)
        rospy.loginfo("finish trajectory")


    def init_pose(self):
        rospy.loginfo("go to init pose")
        r_goal = Transform.from_list([0.2, 0.2, 0.22, 1, 0, 0, 0])
        self.mc.goto_joint_pose('right_arm', r_goal)
        l_goal = Transform.from_list([0.2, -0.2, 0.22, 1, 0, 0, 0])
        self.mc.goto_joint_pose('left_arm', l_goal)
        rospy.loginfo("finish init pose")

    def get_sim_grasp(self):
        T_base_grasp = Transform.from_list([0.35, 0.1, 0.05, 1, 0, 0, 0])
        group_name = 'right_arm'
        cls = 'pcb'
        return group_name, T_base_grasp, cls

    def run_cartesian(self):
        rospy.loginfo("start grasp loop")
        self.init_pose()
        cnt = 1

        while not rospy.is_shutdown() and cnt:
            group_name, grasp, cls = self.get_sim_grasp()
            trajectory1, trajectory2 = self.plan_trajectories(group_name, grasp, cls)
            self.execute_trajectory(group_name, trajectory1)
            self.mc.move_gripper(width=0, force=20)
            self.execute_trajectory(group_name, trajectory2)
            cnt -=1
        rospy.loginfo("finish grasp loop")


def main():
    rospy.init_node("nachi_dual_arm_grasp")
    print('dual arms is starting')
    arm_grasp = NachiGraspController()
    raw_input('please press enter to start grasp loop')
    arm_grasp.run_cartesian()


if __name__ == "__main__":
    main()
