#!/usr/bin/env python
import pickle
from dmp import DMP
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import glob
from matplotlib import pyplot as plt
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
import pickle
import numpy as np

import argparse
from visualization_msgs.msg import Marker


class PlayBack():

    def __init__(self,group):
        rospy.init_node('dmp_playback',anonymous=True)
        self.traj_pub = rospy.Publisher('motion_segment',JointTrajectory,queue_size=1)
        rospy.Subscriber('joint_states',JointState,self.callback)
        self.header = None
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group)

        self.dmp_rollout_pub = rospy.Publisher('dmp_rollout', Marker, queue_size=10)

    def load_dmp(self,i):
        with open('dmp%05d.npy'%i,"r") as f:
            obj = pickle.load(f)
        return obj

    def callback(self,msg):
        self.header = msg.header

    def plotTrajectory(self,trajectory,ns,r,g,b):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.ns = ns
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        for i in range(trajectory.shape[0]):
            point = geometry_msgs.msg.Point()
            point.x = trajectory[i,0]
            point.y = trajectory[i,1]
            point.z = trajectory[i,2]
            marker.points.append(point)

        self.dmp_rollout_pub.publish(marker)

    def spin(self):
        rate = rospy.Rate(0.5)
        dmp_count = 1
        Ndmp = len(glob.glob('./*.npy'))

        
        while not rospy.is_shutdown():
            if not (self.header == None):
                dmp = self.load_dmp(dmp_count)
                print('openning dmp %d'%dmp_count)
                y_r,dy_r,ddy_r = dmp.rollout()
                #print(y_r)
                #print('=========')

                #plt.plot(y_r)                
                #plt.legend(['x', 'y', 'z', 'x_a', 'y_a','z_a', 'w'])
                #plt.show()
                #plt.close()

                self.plotTrajectory(y_r,"dmp_rollout",1,0,0)

                waypoints = []
                for i in range(y_r.shape[0]):
                    pose = geometry_msgs.msg.Pose()
                    pose.position.x = y_r[i,0]
                    pose.position.y = y_r[i,1]
                    pose.position.z = y_r[i,2]
                    pose.orientation.x = y_r[i,3]
                    pose.orientation.y = y_r[i,4]
                    pose.orientation.z = y_r[i,5]
                    pose.orientation.w = y_r[i,6]

                    waypoints.append(copy.deepcopy(pose))

                    
                plan,fraction = self.group.compute_cartesian_path(waypoints,0.01,0.0)
                    

                self.group.execute(plan,wait=True)
                #jtp.positions = y_r[i,:].tolist()
                #jtp.velocities = dy_r[i,:].tolist()
                #jtp.time_from_start = rospy.Duration(1)
                #jt.points.append(jtp)

                #self.traj_pub.publish(jt)

                with open ("./pathF.npy","w") as path_f:
                    pickle.dump(waypoints,path_f,pickle.HIGHEST_PROTOCOL)

                print('Publishing dmp %d'%dmp_count)
                dmp_count += 1

                if dmp_count > Ndmp:
                    break
            rate.sleep()

class SetUp():      
    
    def raise_torso(self,robot):    
        tor = moveit_commander.MoveGroupCommander("torso")
        tor.set_pose_reference_frame("base_link")
        tor.go([0.2])

    def home_right_arm(self,robot):
        mgc = moveit_commander.MoveGroupCommander("right_arm")
        jv = mgc.get_current_joint_values()
        jv[0] = -1

        mgc.set_joint_value_target(jv)
        p = mgc.plan()
        mgc.execute(p)

    def home_left_arm(self,robot):
        
        mgc = moveit_commander.MoveGroupCommander("left_arm")
        mgc.get_current_joint_values()
        jv = mgc.get_current_joint_values()
        jv[0]=1.0

        mgc.set_joint_value_target(jv)
        mgc.plan()
        p = mgc.plan()
        mgc.execute(p)

    def home_head(self):
        mgc = moveit_commander.MoveGroupCommander("head")
        jv = mgc.get_current_joint_values()
        jv[1] = 0.5
        mgc.set_joint_value_target(jv)
        p = mgc.plan()
        mgc.execute(p)

    

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--setup", help="learn with MLN", action="store_true")
    parser.add_argument("-d", "--deploy", help="query MLN", action="store_true")
    args = parser.parse_args()

    if args.setup:
        rospy.init_node("moveit_demo")
        robot = moveit_commander.RobotCommander()
       
        su = SetUp()

        su.raise_torso(robot)
        su.home_right_arm(robot)
        su.home_left_arm(robot)
        su.home_head()

    elif args.deploy:
        pb = PlayBack('right_arm')
        pb.spin()
    else:
        print('please choose s for seting up the robot or d to deploy playing the dmp')
