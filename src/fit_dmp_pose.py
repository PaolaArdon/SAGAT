#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from dmp import DMP
from moveit_msgs.msg import DisplayTrajectory
import matplotlib.pyplot as plt
import pickle

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class Imitate():

    def __init__(self,group):
        rospy.init_node('dmp_fitter',anonymous=True)

	self.name = group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group)

        self.pose_0 = None
        self.pose_1 = None

        self.moving = False
        self.traj = []
        self.dmp_count = 0

    def callback(self,msg):
        if self.pose_0 == None:
            self.pose_0 = [msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w]
            self.pose_1 = [msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w]
        else:
            self.pose_0 = self.pose_1
            self.pose_1 = [msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w]

        if np.sum(np.abs(np.array(self.pose_0) - np.array(self.pose_1))) > 0.01:
            print('Moving')
            self.traj.append(self.pose_1)
        else:
            if len(self.traj) > 5:

                # Fit DMP to motion segment
                path = np.array(self.traj)
                with open ("./path%05d.npy"%self.dmp_count,"w") as path_f:
                    pickle.dump(path,path_f,pickle.HIGHEST_PROTOCOL)
                    
                dmp = DMP(path[0,:],path[-1,:], Nb=500, dt=0.01,
                d=path.shape[1],jnames=self.name)
                params = dmp.imitate_path(path+1e-5*np.random.randn(path.shape[0],path.shape[1]))
                #print(params)

                dmp.reset_state()
                self.dmp_count += 1

                with open ("./dmp%05d.npy"%self.dmp_count,"w") as f:
                    pickle.dump(dmp,f,pickle.HIGHEST_PROTOCOL)

                y_r,dy_r,ddy_r = dmp.rollout()
                plt.subplot(1,2,1)
                plt.cla()
                plt.plot(y_r)
                plt.subplot(1,2,2)
                plt.cla()
                plt.plot(path)
                plt.draw()
                plt.pause(0.1)

            self.traj = []

            print('Stationary')

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose = self.group.get_current_pose().pose
            self.callback(pose)
            r.sleep()


if __name__ == '__main__':
    plt.ion()
    im = Imitate('right_arm')
    im.spin()
