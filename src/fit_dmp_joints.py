#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from dmp import DMP
from moveit_msgs.msg import DisplayTrajectory
import matplotlib.pyplot as plt
import pickle

class Imitate():

    def __init__(self,arm_prefix):
        rospy.init_node('dmp_fitter',anonymous=True)
        rospy.Subscriber('joint_states',JointState,self.callback)

        self.joint_names = [arm_prefix + joint for joint in
        ["_shoulder_pan_joint","_shoulder_lift_joint", "_upper_arm_roll_joint",
        "_elbow_flex_joint", "_forearm_roll_joint", "_wrist_flex_joint",
        "_wrist_roll_joint"]]

        self.js_0 = None
        self.js_1 = None
        self.jt = JointTrajectory()
        self.traj_pub = rospy.Publisher('motion_segment',JointTrajectory,queue_size=1)
        self.moving = False
        self.traj = []
        self.dmp_count = 0

    def callback(self,msg):
        if self.js_0 == None:
            self.js_0 = msg
            self.js_1 = msg
        else:
            self.js_0 = self.js_1
            self.js_1 = msg

        if np.sum(np.abs(np.array(self.js_0.position) -
        np.array(self.js_1.position))) > 0.01:
            print('Moving')
            j_list = []
            for j,joint in enumerate(self.joint_names):
		i = msg.name.index(joint)
		if i:
	        	j_list.append(msg.position[i])
            self.traj.append(j_list)
        else:
            if len(self.traj) > 0:

                # Fit DMP to motion segment
                path = np.array(self.traj)
                dmp = DMP(path[0,:],path[-1,:], Nb=500, dt=0.01,
                d=path.shape[1],jnames=self.joint_names)
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
                self.jt.header = msg.header
                self.jt.joint_names = self.joint_names
                jtp = JointTrajectoryPoint()
                for i in range(y_r.shape[0]):
                    jtp.positions = y_r[i,:].tolist()
                    jtp.velocities = dy_r[i,:].tolist()
                    jtp.time_from_start = rospy.Duration(1.0)
                    self.jt.points.append(jtp)
                self.traj_pub.publish(self.jt)

                self.traj = []

            print('Stationary')
            self.jt = JointTrajectory()

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    plt.ion()
    im = Imitate('r')
    im.spin()
