#!/usr/bin/env python2.7

from __future__ import print_function

import rospy

import sys
import copy
from math import *
import moveit_commander
import numpy as np

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, PoseArray
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from niryo_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan

"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles):
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)


"""
    Creates a pick and place plan using the four states below.

    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""



def near_object(request):

    result0 = sqrt((request.pick_pose.poses[0].position.x)**2+(request.pick_pose.poses[0].position.y)**2+(request.pick_pose.poses[0].position.z)**2)
    distEuclid = PoseArray()
    distEuclid.poses.append(request.pick_pose.poses[0])
    results = [result0]
    print(distEuclid)
    for i in range(1,len(request.pick_pose.poses)):

        resultsCopy = []
        result = sqrt((request.pick_pose.poses[i].position.x)**2+(request.pick_pose.poses[i].position.y)**2+(request.pick_pose.poses[i].position.z)**2)
        isNotEmpty = False

        for j in range(len(distEuclid.poses)):
            while isNotEmpty == False:
                if (len(distEuclid.poses) == 1):
                    print('if1')
                    if (result < results[j]):
                        print('if11')
                        distEuclid.poses.insert(0,request.pick_pose.poses[i])
                        resultsCopy = [result]+ results[0:len(results)]
                        isNotEmpty = True

                    else:
                        print('if12')
                        distEuclid.poses.append(request.pick_pose.poses[i])
                        resultsCopy = [results[0:len(results)]]+[result]
                        isNotEmpty = True



                else:
                    print('if2')
                    print(result)
                    print("redftgft")
                    print(type(distEuclid[0]))


                    if (result <results[j]):
                        print('if21')
                        distEuclid.poses.insert(j,request.pick_pose.poses[i])
                        resultsCopy=resultsCopy[0:j-1]+[result]+resultsCopy[j:len(results)]
                        isNotEmpty = True


                    elif (result >= results[-1]):
                        print('if23')
                        distEuclid.poses.append(request.pick_pose.poses[i])
                        resultsCopy=resultsCopy+[result]
                        isNotEmpty = True
                pass
            print(isNotEmpty)

    # for i in range(len(distEuclid)):
    #
    #     x = distEuclid[i].position.x;
    #     y = distEuclid[i].position.y;
    #     z = distEuclid[i].position.z;
    #
    #     qx = distEuclid[i].orientation.x;
    #     qy = distEuclid[i].orientation.y;
    #     qz = distEuclid[i].orientation.z;
    #     qw = distEuclid[i].orientation.w;
    #
    #
    #     posX = float(x);
    #     posY = float(y);
    #     posZ = float(z);
    #     quatX = float(qx);
    #     quatY = float(qy);
    #     quatZ = float(qz);
    #     quatW = float(qw);
    #
    #     objets = Pose()
    #
    #     objets.position.x =posX;
    #     objets.position.y=posY;
    #     objets.position.z=posZ;
    #
    #     objets.orientation.x=quatX;
    #     objets.orientation.y = quatY;
    #     objets.orientation.z = quatZ;
    #     objets.orientation.w = quatW;
    #
    #     listObjects = []
    #     listObjects += objets


    print(distEuclid)
    return distEuclid




def plan_pick_and_place(req):
    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    near_object1 = near_object(req)

    trajectoires = []

    for i in range(len(near_object1.poses)):

        # Pre grasp - position gripper directly above target object
        pre_grasp_pose = plan_trajectory(move_group, near_object1.poses[i], current_robot_joint_configuration)
        print(pre_grasp_pose)
        # If the trajectory has no points, planning has failed and we return an empty response
        if not pre_grasp_pose.joint_trajectory.points:
            return response

        previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(near_object1.poses[i])
        pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
        grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)

        if not grasp_pose.joint_trajectory.points:
            return response

        previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

        # Pick Up - raise gripper back to the pre grasp position
        pick_up_pose = plan_trajectory(move_group, near_object1.poses[i], previous_ending_joint_angles)

        if not pick_up_pose.joint_trajectory.points:
            return response

        previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

        # Place - move gripper to desired placement position
        place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

        if not place_pose.joint_trajectory.points:
            return response

        previous_ending_joint_angles = place_pose.joint_trajectory.points[-1].positions


        # Init  - move gripper to the initial position
        # reset_Pose = Pose()
        # reset_Pose.position.x = 0
        # reset_Pose.position.y = 0
        # reset_Pose.position.z = 0
        # reset_Pose.orientation.x = 0
        # reset_Pose.orientation.y = 0
        # reset_Pose.orientation.z = 0
        # reset_Pose.orientation.w = 0

        current_robot_joint_configuration

        init_pose = plan_trajectory(move_group, previous_ending_joint_angles, req.joints_input.joints)

        if not init_pose.joint_trajectory.points:
            return response






        # If trajectory planning worked for all pick and place stages, add plan to response
        response.trajectories.append(pre_grasp_pose)
        response.trajectories.append(grasp_pose)
        response.trajectories.append(pick_up_pose)
        response.trajectories.append(place_pose)

        #Clear all known pose targets
        move_group.clear_pose_targets()

        trajectoires.append(response.trajectories)

        print(type(response.trajectories))

    return trajectoires


def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('niryo_moveit_server')

    s = rospy.Service('niryo_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()
