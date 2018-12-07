#!/usr/bin/env python
# license removed for brevity


# This file is a basic structure to write a controller that
# communicates with ROS. It will be the students responsibility
# tune the gains and fill in the missing information

# As an example this file contains PID gains, but other
# controllers use different types of gains so the class
# will need to be modified to accomodate those changes

import rospy
import time
import math
from whirlybird_msgs.msg import Command
from whirlybird_msgs.msg import Whirlybird
from whirlybird_msgs.msg import EstStates
from std_msgs.msg import Float32
import numpy as np
import os
import control


class Observer():

    def __init__(self):

        # get parameters
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')
        print("Start of Observable!!!!")
        self.lonState = np.zeros((2,1))
        self.latState = np.zeros((4,1))
        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']
        self.Fe = (m1*l1-m2*l2)*g*math.cos(0)/l1 #Note this is not the correct value for Fe, you will have to find that yourself
        print("Next 1")
        #SS Stuff
        self.Alat = np.array([[0., 0., 1., 0.], [0., 0., 0., 1], [0., 0., 0., 0.], [(l1*self.Fe)/(m1*l1**2+m2*l2**2+Jz), 0., 0., 0.]])
        self.Blat = np.array([[0.],[0.],[1/Jx],[0.]])
        self.Blat = self.Blat.reshape((4,1))
        self.Alat = self.Alat.reshape((4,4))
        self.Clat = np.array([[1.,0.,0.,0.],[0., 1., 0., 0.]])
        self.Clat = self.Clat.reshape((2,4))
        print("Next 2")
        O = control.obsv(self.Alat,self.Clat)
        print(self.Alat)
        print(self.Clat)
        print(O)
        print("Next 3")
        contrable = np.rank(O)
        print("Next 4")
        print(contrable)
        if (contrable!=4):
            print("Lat Not Observable!!!!")
        else:
            print("Lat Observable")
        print("Next 5")
        C_yaw = np.array([0., 1., 0., 0.])
        C_yaw = C_yaw.reshape((1,4))


        # self.Alon = np.array([[0., 1., 0.],[0., 0., 0.],[-1.,0.,0.]])
        # self.Blon = np.array([[0],[(l1/(m1*l1**2+m2*l2**2+Jy))],[0.]])
        self.Alon = np.array([[0., 1.],[0., 0.]])
        self.Blon = np.array([[0.],[(l1/(m1*l1**2+l2**2+Jy))]])
        self.Clon = np.array([1.,0.])
        self.Alon = self.Alon.reshape((2,2))
        self.Clon = self.Clon.reshape((1,2))
        print(self.Alon)
        print(self.Clon)
        Ctrlon = control.obsv(self.Alon,self.Clon)
        print(Ctrlon)
        contrable = np.rank(Ctrlon)
        if (contrable!=2):
            print("Lon Not Observable!!!!")
        else:
            print("Lon Observable")




        # Roll Gains
        zeta_p = .707 #.707
        tr_p = .25
        omegan_p = 2.2/tr_p

        # Pitch Gains
        b0_th = 1.152
        zeta_th = .707 #.707
        tr_th = .9 #1.4
        omegan_th = 2.2/tr_th

        # Yaw Gains
        b_psi = (l1*self.Fe)/(m1*l1**2 + m2*l2**2 + Jz) #Works better for l2 than l1 even though it should be l1
        zeta_psi = .707#.707
        m_psi = 10#8
        tr_psi = m_psi*tr_p
        omegan_psi = 2.2/tr_psi



        #observer gains
        roll_gain_obs = omegan_p*10
        pitch_gain_obs = omegan_th*5.5
        yaw_gain_obs = omegan_psi*10

        print("Yo")
        #Lat Poles
        # p_lat_int = -omegan_psi/2.0
        p_lat_int = -1.1
        print(p_lat_int)
        polynomialLat= np.convolve([1, 2*zeta_p*roll_gain_obs, roll_gain_obs**2], [1, 2*zeta_psi*yaw_gain_obs, yaw_gain_obs**2])
        print(polynomialLat)
        plat = np.roots(polynomialLat)
        print(plat)
        self.L_lat = np.transpose(np.array(control.place(np.transpose(self.Alat), np.transpose(self.Clat), plat)))
        print(self.L_lat)
        print("Bradford")


        polynomialLon = np.array([1, 2*zeta_th*pitch_gain_obs, pitch_gain_obs**2])
        print(polynomialLon)
        plon = np.roots(polynomialLon)
        print(plon)
        self.L_lon = np.transpose(np.array(control.place(np.transpose(self.Alon), np.transpose(self.Clon), plon)))
        print("Yo4")
        print(self.L_lon)

        # print(plat)
        # print(plong)

        #ROLL
        # self.P_phi_ = omegan_p**2*Jx
        # self.I_phi_ = 1
        # self.D_phi_ = 2*zeta_p*omegan_p*Jx
        self.Int_phi = 0.0
        self.prev_phi = 0.0
        self.error_phi_prev = 0.0
        self.phid = 0.0

        # PITCH
        self.theta_r = 0.0
        # self.P_theta_ = omegan_th**2/b0_th
        # self.I_theta_ = 1.0
        # self.D_theta_ = (2*zeta_th*omegan_th)/b0_th
        self.prev_theta = 0.0
        self.Int_theta = 0.0
        self.error_theta_prev = 0.0
        self.thetad = 0.0

        # YAW
        self.psi_r = 0.0
        # self.P_psi_ = omegan_psi**2/b_psi
        # self.I_psi_ = .1
        # self.D_psi_ = (2*zeta_psi*omegan_psi)/b_psi
        self.prev_psi = 0.0
        self.Int_psi = 0.0
        self.psid = 0.0
        self.error_psi_prev = 0.0


        self.prev_time = rospy.Time.now()
        self.sigma = .05

        self.command_r = 0.0
        self.command_l = 0.0

        self.command_sub_ = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybirdCallback, queue_size=5)
        # self.psi_r_sub_ = rospy.Subscriber('psi_r', Float32, self.psiRCallback, queue_size=5)
        # self.theta_r_sub_ = rospy.Subscriber('theta_r', Float32, self.thetaRCallback, queue_size=5)
        self.command_sub_ = rospy.Subscriber('command', Command, self.commandCallback, queue_size=5)
        self.estStates_pub_ = rospy.Publisher('estStates', EstStates, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    # def thetaRCallback(self, msg):
    #     self.theta_r = msg.dataEstStates()
    #
    #
    # def psiRCallback(self, msg):
    #     self.psi_r = msg.data

    def commandCallback(self, msg):
        self.command_r = msg.right_motor
        self.command_l = msg.left_motor


    def whirlybirdCallback(self, msg):
        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']

        F = (self.command_r + self.command_l)*km
        Tau = (self.command_l*d - self.command_r*d)*km


        phi = msg.roll
        theta = msg.pitch
        psi = msg.yaw

        latmeasuredStates = np.array([[phi],[psi]])
        Fe = (m1*l1-m2*l2)*g*math.cos(0)/l1

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now

        ##################################
        # Implement your observer here
        # print(self.state[0,2,4,5])
        # print("here")
        # print(self.Alon)
        # print(self.lonState)
        # print(np.matmul(self.Alon,self.lonState))
        # # print(np.matmul(self.L_lon,(theta - np.matmul(self.Clon,self.lonState))))
        # print(dt)
        # print(theta)
        # print(self.Blon)
        # print(self.Blon*(F-Fe))
        # print(F)
        # print(self.Clon)
        N = 10
        for i in range(0,N):
            # print(self.lonState)
            # print((theta - np.matmul(self.Clon,self.lonState)))
            # print(np.matmul(self.L_lon,(theta - np.matmul(self.Clon,self.lonState))))
            self.latState = self.latState + (dt/N)*((np.matmul(self.Alat,self.latState)) + self.Blat*(Tau) + np.matmul(self.L_lat,(latmeasuredStates - np.matmul(self.Clat,self.latState))))
            self.lonState = self.lonState + (dt/N)*((np.matmul(self.Alon,self.lonState)) + self.Blon*(F-Fe) + np.matmul(self.L_lon,(theta - np.matmul(self.Clon,self.lonState))))
            # self.lonState = self.lonState + (dt/N)*(self.Alon*self.lonState) + self.Blon*(F-Fe) + self.L_lon*(theta - (self.Clon*self.lonState))


            # print(self.latState)
            # print(self.lonState)

        # Pack up and send command
        estimatedStates = EstStates()
        print(self.lonState)
        estimatedStates.roll = float(self.latState[0])
        estimatedStates.pitch = float(self.lonState[0])
        estimatedStates.yaw = float(self.latState[1])
        estimatedStates.rolld = float(self.latState[2])
        estimatedStates.pitchd = float(self.lonState[1])
        estimatedStates.yawd = float(self.latState[3])
        self.estStates_pub_.publish(estimatedStates)


if __name__ == '__main__':
    rospy.init_node('observer', anonymous=True)
    try:
        observer = Observer()
    except:
        rospy.ROSInterruptException
    pass
