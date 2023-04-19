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
            #print(isNotEmpty)
    return distEuclid




# This algorithm has been tested and it works
def near_object_V2(req):
    # distEuclid0 = sqrt((req.pick_pose.poses[0].position.x)**2 + (req.pick_pose.poses[0].position.y)**2 + (req.pick_pose.poses[0].position.z)**2)
    orList = req.pick_pose.poses
    print(orList)
    # sortedList = [req.pick_pose.poses[0]]
    for i in range(0,len(orList)):
        distEuclid_i = sqrt((req.pick_pose.poses[i].position.x)**2 + (req.pick_pose.poses[i].position.y)**2 + (req.pick_pose.poses[i].position.z)**2)
        for j in range(i+1,len(orList)):
            distEuclid_j = sqrt((req.pick_pose.poses[j].position.x)**2 + (req.pick_pose.poses[j].position.y)**2 + (req.pick_pose.poses[j].position.z)**2)
            if (distEuclid_i >= distEuclid_j):
                orList[i],orList[j] = orList[j], orList[i]
            j+=1
    sortedList = orList

    #print(sortedList)
    return sortedList





def plan_pick_and_place(req):
    response = MoverServiceResponse()
    # Here we sort our list
    near_object_V2(req)

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Index of the thingy that will take over the element of type (cubes == PoseArray)
    # i = 0
    print("jjjj")
    trajectoires = []

    for i in range (len(req.pick_pose.poses)):
        print("#####")
        print(i)
        print("#####")

        # Current robot joint configuration initial haha
        current_robot_joint_configuration = req.joints_input.joints

        # Pre grasp - position gripper directly above target object
        pre_grasp_pose = plan_trajectory(move_group, req.pick_pose.poses[i], current_robot_joint_configuration)

        # If the trajectory has no points, planning has failed and we return an empty response
        if not pre_grasp_pose.joint_trajectory.points:
            return response
        print("12345ttttt\n")
        previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose.poses[i])
        pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
        grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
        print("12345aaaaa\n")
        # If the trajectory has no points, planning has failed and we return an empty response
        if not grasp_pose.joint_trajectory.points:
            print("No trajectory found ", i)
            return response

        previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

        # Pick Up - raise gripper back to the pre grasp position
        pick_up_pose = plan_trajectory(move_group, req.pick_pose.poses[i], previous_ending_joint_angles)
        # If the trajectory has no points, planning has failed and we return an empty response
        if not pick_up_pose.joint_trajectory.points:
            print("No trajectory found ", i)
            return response

        previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions
        # Place - move gripper to desired placement position
        place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

        # If the trajectory has no points, planning has failed and we return an empty response
        if not place_pose.joint_trajectory.points:
            print("No trajectory found ", i)
            return response

        # If trajectory planning worked for all pick and place stages, add plan to response
        response.trajectories.append(pre_grasp_pose)
        response.trajectories.append(grasp_pose)
        response.trajectories.append(pick_up_pose)
        response.trajectories.append(place_pose)

        trajectoires.append(response.trajectories)

        print("###################################################################")
        print(response)
        print(len(trajectoires))
        print("###################################################################")


        move_group.clear_pose_targets()






    # print("############################################################################")
    # print(trajectoires)
    # print("############################################################################")
    #
    # for k in range(len(trajectories)):
    #     print("#####")
    #     print(k)
    #     print("#####")
    #     return trajectories[k]



def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('niryo_moveit_server')

    s = rospy.Service('niryo_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()
