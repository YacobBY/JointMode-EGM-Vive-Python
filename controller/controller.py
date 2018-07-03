from Vive import Vive_thread
import time
import os,sys
import numpy as np
from robot_egm import Robot_EGM
import threading
from math import sqrt
import ABB120
from math3D import *

class control_thread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self.J=np.zeros(6)
        self.vive=Vive_thread()
        self.egm=Robot_EGM()
        os.system('cls')
        print('********')
        self.vive.connect()
        connection_status=self.egm.connect()
        print("EGM connection:  ", connection_status)
        print('********')

        self.track_started = False
        self.Rcr=np.eye(3)
        self.Htr_desired=np.eye(4)
        self.Desired_J=[0,0,0,0,0,0]

        self.controller_run_flag=True
        self.grip_button_pressed=False
        self.controller_except_flag=False

        self.command_J=[0,0,0,30,0,60]

        self.p_gain=0.09
        self.d_gain=0.00


        self.J_offset_sat=1

        self.J_home=np.array([0, 0, 30, 0, 60, 0])
        self.J=self.J_home
        self.J_last=self.J

        H_tool,xyz,R,Q=ABB120.FK(self.J_home)
        self.H_home=H_tool
        self.Rtr_home=H_tool[0:3,0:3]
        self.X_home=xyz

        self.J_max =np.array([ 90,  80,  55,  90,  90,  150])
        self.J_min =np.array([-90, -30, -50, -90, -90, -150])

        #  Calibration XYZ

        a1=np.array([-962.2, -521.9, 383.5])
        a2=np.array([ -81.2, -489.1, 386.6])
        a3=np.array([-965.9,  388.2, 397.9])

        vx=a1-a3
        vy=a2-a1
        vz=np.cross(vx,vy)

        self.Rvr=np.eye(3)

        self.Rvr[:,0] = vx/sqrt(vx[0]**2 + vx[1]**2 + vx[2]**2)
        self.Rvr[:,1] = vy/sqrt(vy[0]**2 + vy[1]**2 + vy[2]**2)
        self.Rvr[:,2] = vz/sqrt(vz[0]**2 + vz[1]**2 + vz[2]**2)

        self.Rcv_home =np.array([[-0.998827  , -0.048421  ,  -0.000450],
                                 [-0.018378  ,  0.387672  ,  -0.921614],
                                 [ 0.044800  , -0.920525  ,  -0.388107]])

        self.RRvr =np.array([[ 0 , -1 , 0 ],
                             [ 0 ,  0 , 1 ],
                             [-1 ,  0 , 0 ]])

        self.Rccnew=np.matmul(np.transpose(self.Rcv_home),self.RRvr)
        self.Xcv = 0
        self.Xcr = 0

    def stop(self):
        #self._stop_event.set()
        self.controller_run_flag=False
        self.vive.stop()

    def run(self):
        try:
            self.vive.start()
            while self.controller_run_flag:
                time.sleep(0.001)
                self.egm.loop()
                P=self.vive.getPose()
                d=self.vive.getState()
                self.J=self.egm.getJoints()
                self.command_J = self.J

                if (len(d)==0):
                    self.track_started=False
                    self.set_zero_offset()
                    self.egm.setJoints(self.command_J)
                    continue

                self.grip_button_pressed=d['grip_button']
                self.Xcv = np.array([-P[0,3]*1000, P[2,3]*1000, P[1,3]*1000])
                self.Xcr = np.matmul(np.transpose(self.Rvr),self.Xcv)

                self.Rcv=np.eye(3)
                self.Rcv=P[0:3,0:3]
                self.Rcr=np.matmul(np.matmul(np.transpose(self.RRvr),self.Rcv),self.Rccnew);

                if (self.grip_button_pressed):
                    if (not self.track_started):
                        self.track_started=True
                        self.J_start = self.J
                        self.Xcr_start = self.Xcr

                    self.Xcr_relative = self.Xcr-self.Xcr_start
                    self.calculate_offset()
                    self.egm.setJoints(self.command_J)
                else:
                    self.track_started=False
                    self.set_zero_offset()
                    self.egm.setJoints(self.command_J)
                self.J_last=self.J

        except Exception as e:
            self.controller_except_flag=True
            self.exception=e

    def calculate_offset(self):
        H_tool,xyz,R,Q=ABB120.FK(self.J)

        self.Rtr=H_tool[0:3,0:3]
        self.Xtr=xyz

        #H_tool,xyz,R,Q=ABB120.FK(self.J_start)
        #self.Xtr_start= xyz
        #self.Xtr_relative=  self.Xtr-self.Xtr_start

        #********************************************************************
        #********************************************************************
        Htr_desired=np.eye(4)
        #self.Htr_desired[0:3,0:3]=self.Rtr_home
        Htr_desired[0:3,0:3]=np.matmul(self.Rcr,self.Rtr_home)
        #self.Htr_desired[0:3,0:3]=np.array([[0,0,1],
        #                                    [1,0,0],
        #                                    [0,1,0]])
        Xtr_desired = self.Xcr_relative + self.X_home
        #Xtr_desired = self.X_home
        Htr_desired[0:3,3] = Xtr_desired

        IK_results = ABB120.IK(Htr_desired)
        #print(Htr_desired)
        #print(IK_results)
        if IK_results is not None:
            #print("IK_results is not None")
            self.Desired_J=IK_results[0]
            self.command_J=self.check_joint_limits(self.Desired_J)

    def check_joint_limits(self,J_desired):
        J_checked = J_desired
        if (J_desired[0]>self.J_max[0]): J_checked[0]=self.J[0]
        if (J_desired[0]<self.J_min[0]): J_checked[0]=self.J[0]
        if (J_desired[1]>self.J_max[1]): J_checked[1]=self.J[1]
        if (J_desired[1]<self.J_min[1]): J_checked[1]=self.J[1]
        if (J_desired[2]>self.J_max[2]): J_checked[2]=self.J[2]
        if (J_desired[2]<self.J_min[2]): J_checked[2]=self.J[2]
        if (J_desired[3]>self.J_max[3]): J_checked[3]=self.J[3]
        if (J_desired[3]<self.J_min[3]): J_checked[3]=self.J[3]
        if (J_desired[4]>self.J_max[4]): J_checked[4]=self.J[4]
        if (J_desired[4]<self.J_min[4]): J_checked[4]=self.J[4]
        if (J_desired[5]>self.J_max[5]): J_checked[5]=self.J[5]
        if (J_desired[5]<self.J_min[5]): J_checked[5]=self.J[5]

        #if (self.command_J[0]-self.J[0])> self.J_offset_sat: J_checked[0]= self.J_offset_sat
        #if (self.command_J[0]-self.J[0])<-self.J_offset_sat: J_checked[0]=-self.J_offset_sat
        #if (self.command_J[1]-self.J[1])> self.J_offset_sat: J_checked[1]= self.J_offset_sat
        #if (self.command_J[1]-self.J[1])<-self.J_offset_sat: J_checked[1]=-self.J_offset_sat
        #if (self.command_J[2]-self.J[2])> self.J_offset_sat: J_checked[2]= self.J_offset_sat
        #if (self.command_J[2]-self.J[2])<-self.J_offset_sat: J_checked[2]=-self.J_offset_sat
        #if (self.command_J[3]-self.J[3])> self.J_offset_sat: J_checked[3]= self.J_offset_sat
        #if (self.command_J[3]-self.J[3])<-self.J_offset_sat: J_checked[3]=-self.J_offset_sat
        #if (self.command_J[4]-self.J[4])> self.J_offset_sat: J_checked[4]= self.J_offset_sat
        #if (self.command_J[4]-self.J[4])<-self.J_offset_sat: J_checked[4]=-self.J_offset_sat
        #if (self.command_J[5]-self.J[5])> self.J_offset_sat: J_checked[5]= self.J_offset_sat
        #if (self.command_J[5]-self.J[5])<-self.J_offset_sat: J_checked[5]=-self.J_offset_sat

        return J_checked

    def set_zero_offset(self):
        self.command_J=self.J
        self.egm.setJoints(self.command_J)
