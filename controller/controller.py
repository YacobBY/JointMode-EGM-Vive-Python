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

        self.controller_run_flag=True
        self.grip_button_pressed=False
        self.controller_except_flag=False

        self.tool_offset_X=[0,0,0]
        self.tool_offset_Q=[1,0,0,0]

        self.p_gain=0.09
        self.d_gain=0.00
        self.p_gain_angle=0.05
        self.xyz_delta=50
        self.Xtr_error_last=0

        self.offset_sat=4
        self.q_sat=2

        self.J_home=np.array([0, 0, 40, 0, 50, 0])
        H_tool,xyz,R,Q=ABB120.FK(self.J_home)
        self.start_H=H_tool
        self.Rtr_home=self.start_H [0:3,0:3]

        self.J_max =np.array([0, 0,  55,  90, 90, 0])
        self.J_min =np.array([0, 0, -50, -90, 30, 0])

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

                if (len(d)==0):
                    self.track_started=False
                    self.set_zero_offset()
                    self.egm.setOffset(self.tool_offset_X,self.tool_offset_Q)
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
                    self.egm.setOffset(self.tool_offset_X,self.tool_offset_Q)
                else:
                    self.track_started=False
                    self.set_zero_offset()
                    self.egm.setOffset(self.tool_offset_X,self.tool_offset_Q)
        except Exception as e:
            self.controller_except_flag=True
            self.exception=e

    def calculate_offset(self):
        H_tool,xyz,R,Q=ABB120.FK(self.J)

        self.Rtr=H_tool[0:3,0:3]
        self.Xtr=xyz

        H_tool,xyz,R,Q=ABB120.FK(self.J_start)
        self.Xtr_start= xyz

        self.Xtr_relative=  self.Xtr-self.Xtr_start
        self.Xtr_error=self.Xcr_relative-self.Xtr_relative
        #self.Xtr_error_d=(self.Xtr_error-self.Xtr_error_last)/0.001;
        #self.Xtr_error_last=self.Xtr_error;

        self.Xtr_offset=self.Xtr_error*self.p_gain#+self.Xtr_error_d*self.d_gain

        if (self.Xtr[2]<170 and self.Xtr_offset[2]<0):
             self.Xtr_offset[2]=0
        offset=np.matmul(np.transpose(self.Rtr),self.Xtr_offset)

        if (offset[0]> self.xyz_delta):
            offset[0]= self.xyz_delta
        if (offset[0]< -self.xyz_delta):
            offset[0]= -self.xyz_delta

        if (offset[1]> self.xyz_delta):
            offset[1]= self.xyz_delta
        if (offset[1]< -self.xyz_delta):
            offset[1]= -self.xyz_delta

        if (offset[2]> self.xyz_delta):
            offset[2]= self.xyz_delta
        if (offset[2]< -self.xyz_delta):
            offset[2]= -self.xyz_delta

        #********************************************************************
        #********************************************************************
        new_tool=np.transpose(self.Rtr_home)
        Rtr_desired=self.Rcr;
        current_R=np.matmul(self.Rtr,new_tool)
        d_R=np.matmul(np.transpose(current_R),Rtr_desired)
        d_aa_angle, d_aa_axis=angle_axis(d_R);
        d_aa_small=rotation_matrix(d_aa_angle*self.p_gain_angle,np.matmul(np.transpose(self.Rtr),d_aa_axis));
        d_q=q_from_R(d_aa_small)

        # SINGULARITY CHECK*********************
        '''
        Htr_desired=np.zeros((4,4))
        Htr_desired[0:3,0:3]=np.matmul(Rtr_desired,np.transpose(new_tool))

        Xrt_desired = self.Xcr_relative+self.Xtr_start
        Htr_desired[0:3,0:3] = Xrt_desired

        IK_results = ABB120.IK(Htr_desired)
        IK_result=IK_results[0]

        if (IK_result):
            if (     (J[2]>J_max[2]) and (J_desired[2]>J_max[2])
                  or (J[2]<J_min[2]) and (J_desired[2]<J_min[2])
                  or (J[3]>J_max[3]) and (J_desired[3]>J_max[3])
                  or (J[3]<J_min[3]) and (J_desired[3]<J_min[3])
                  or (J[4]>J_max[4]) and (J_desired[4]>J_max[4])
                  or (J[4]<J_min[4]) and (J_desired[4]<J_min[4])
                  ):
                print( "OUT OF RANGE!!!" )
                set_zero_offset()
            elif (     (J_desired[2]>J_max[2])
                       or (J_desired[2]<J_min[2])
                       or (J_desired[3]>J_max[3])
                       or (J_desired[3]<J_min[3])
                       or (J_desired[4]>J_max[4])
                       or (J_desired[4]<J_min[4])
                       ):
                print( "OUT OF RANGE!!!" )
            else:
                '''
        if True:
            if True:
                if (offset[0]>self.offset_sat):
                    self.tool_offset_X[0]=self.offset_sat
                elif (offset[0]<-self.offset_sat):
                    self.tool_offset_X[0]=-self.offset_sat
                else:
                    self.tool_offset_X[0]=offset[0]

                if (offset[1]>self.offset_sat):
                    self.tool_offset_X[1]=self.offset_sat
                elif (offset[1]<-self.offset_sat):
                    self.tool_offset_X[1]=-self.offset_sat
                else:
                    self.tool_offset_X[1]=offset[1]


                if (offset[2]>self.offset_sat):
                    self.tool_offset_X[2]=self.offset_sat;
                elif (offset[2]<-self.offset_sat):
                    self.tool_offset_X[2]=-self.offset_sat
                else:
                    self.tool_offset_X[2]=offset[2]


                if (d_q[0]>self.q_sat):
                    self.tool_offset_Q[0]=self.q_sat
                elif (d_q[0]<-self.q_sat):
                    self.tool_offset_Q[0]=-self.q_sat
                else:
                    self.tool_offset_Q[0]=d_q[0]

                if (d_q[1]>self.q_sat):
                    self.tool_offset_Q[1]=self.q_sat
                elif (d_q[1]<-self.q_sat):
                    self.tool_offset_Q[1]=-self.q_sat
                else:
                    self.tool_offset_Q[1]=d_q[1]

                if (d_q[2]>self.q_sat):
                    self.tool_offset_Q[2]=self.q_sat
                elif (d_q[2]<-self.q_sat):
                    self.tool_offset_Q[2]=-self.q_sat
                else:
                    self.tool_offset_Q[2]=d_q[2]

                if (d_q[3]>self.q_sat):
                    self.tool_offset_Q[3]=self.q_sat
                elif (d_q[3]<-self.q_sat):
                    self.tool_offset_Q[3]=-self.q_sat
                else:
                    self.tool_offset_Q[3]=d_q[3]
        else:
            self.set_zero_offset()

    def set_zero_offset(self):
        self.tool_offset_X=[0,0,0]
        self.tool_offset_Q=[1,0,0,0]
