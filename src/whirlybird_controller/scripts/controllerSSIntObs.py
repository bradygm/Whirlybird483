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
from whirlybird_msgs.msg import EstStates
from std_msgs.msg import Float32
import numpy as np
import os
import control


class Controller():

    def __init__(self):

        # get parameters
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')

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

        #SS Stuff
        self.Alat = np.array([[0., 0., 1., 0., 0.], [0., 0., 0., 1,0.], [0., 0., 0., 0.,0.], [(l1*self.Fe)/(m1*l1**2+m2*l2**2+Jz), 0., 0., 0.,0.],
                                [0., -1., 0., 0.,0.]])
        self.Blat = np.array([[0.],[0.],[1/Jx],[0.],[0.]])
        self.Blat = self.Blat.reshape((5,1))
        self.Alat = self.Alat.reshape((5,5))
        C = control.ctrb(self.Alat,self.Blat)
        contrable = np.linalg.det(C)
        if (contrable==0):
            print("Lat Not Controlable!!!!")
        else:
            print("Lat Controllable")
        C_yaw = np.array([0., 1., 0., 0.])
        C_yaw = C_yaw.reshape((1,4))


        self.Alon = np.array([[0., 1., 0.],[0., 0., 0.],[-1.,0.,0.]])
        self.Blon = np.array([[0],[(l1/(m1*l1**2+m2*l2**2+Jy))],[0.]])

        Clon = control.ctrb(self.Alon,self.Blon)
        contrable = np.linalg.det(Clon)
        if (contrable==0):
            print("Lon Not Controlable!!!!")
        else:
            print("Lon Controllable")
        C_lon = np.array([1.,0.])



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

        print("Yo")
        #Lat Poles
        # p_lat_int = -omegan_psi/2.0
        p_lat_int = -1.1
        print(p_lat_int)
        plat = np.array([(-zeta_p*omegan_p+1j*omegan_p*math.sqrt(1.-zeta_p**2.)),
                         (-zeta_p*omegan_p-1j*omegan_p*math.sqrt(1.-zeta_p**2.)),
                         (-zeta_psi*omegan_psi+1j*omegan_psi*math.sqrt(1.-zeta_psi**2.)),
                         (-zeta_psi*omegan_psi-1j*omegan_psi*math.sqrt(1.-zeta_psi**2.)),
                         p_lat_int])
        self.K1_lat = control.place(self.Alat, self.Blat, plat)
        self.K1_lat = self.K1_lat.reshape((1,5))
        print(self.K1_lat)
        # print("Yo2")
        self.K_lat = self.K1_lat[0,0:4]
        print(self.K_lat)
        # inbetween = np.linalg.inv(self.Alat-self.Blat*self.K_lat)
        self.ki_lat = self.K1_lat[0,4]

        print(self.ki_lat)
        #Long Poles
        p_lon_int = -omegan_th/2

        plong = np.array([(-zeta_th*omegan_th+1j*omegan_th*math.sqrt(1-zeta_th**2)),
                         (-zeta_th*omegan_th-1j*omegan_th*math.sqrt(1-zeta_th**2)),
                         p_lon_int])
        self.K1_lon = control.place(self.Alon, self.Blon, plong)
        self.K_lon = self.K1_lon[0,0:2]
        # inbetween = np.linalg.inv(self.Alon-self.Blon*self.K_lon)
        self.ki_lon = self.K1_lon[0,2]
        print("Yo4")
        print(self.K_lon)
        print(self.ki_lon)
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



        self.command_sub_ = rospy.Subscriber('estStates', EstStates, self.whirlybirdCallback, queue_size=5)
        self.psi_r_sub_ = rospy.Subscriber('psi_r', Float32, self.psiRCallback, queue_size=5)
        self.theta_r_sub_ = rospy.Subscriber('theta_r', Float32, self.thetaRCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('command', Command, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def thetaRCallback(self, msg):
        self.theta_r = msg.data


    def psiRCallback(self, msg):
        self.psi_r = msg.data


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

        phi = msg.roll
        theta = msg.pitch
        psi = msg.yaw

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now

        ##################################
        # Implement your controller here
        # a1 = (2.0*self.sigma - dt)/(2*self.sigma + dt)
        # a2 = (2.0/(2*self.sigma+dt))
        # self.thetad = a1*self.thetad+a2*(theta-self.prev_theta)
        # self.psid = a1*self.psid+a2*((psi-self.prev_psi))
        self.thetad = msg.pitchd
        self.psid = msg.yawd

        # self.thetad = (theta-self.prev_theta)/dt
        # self.psid = (psi-self.prev_psi)/dt
        # self.phid = (phi-self.prev_phi)/dt

        self.prev_theta = theta
        self.prev_psi = psi


        self.Int_theta = self.Int_theta + (dt/2)*(self.theta_r-theta + self.error_theta_prev)
        self.error_theta_prev = self.theta_r-theta

        lon_error = 0
        lat_error = 0

        # Ftilde = self.P_theta_*(self.theta_r-theta)-self.D_theta_*self.thetad + self.I_theta_*self.Int_theta
        Ftilde = -np.matmul(self.K_lon,np.array([[theta],[self.thetad]]))-self.ki_lon*self.Int_theta + lon_error
        # print(Ftilde)
        Fe = (m1*l1-m2*l2)*g*math.cos(theta)/l1
        F = Fe + Ftilde
        # print(F)
        self.Int_psi = self.Int_psi + (dt/2)*(self.psi_r-psi + self.error_psi_prev)

        # phi_r = self.P_psi_*(self.psi_r-psi)-self.D_psi_*self.psid +self.I_psi_*self.Int_psi

        # self.phid = a1*self.phid+a2*(phi-self.prev_phi)
        self.phid = msg.rolld

        self.prev_phi = phi
        # self.error_phi_prev = phi_r-phi
        self.error_psi_prev = self.psi_r-psi

        # Tau = self.P_phi_*(phi_r-phi)-self.D_phi_*self.phid #+self.I_phi_*self.Int_phi
        Tau = -np.matmul(self.K_lat,np.array([[phi],[psi],[self.phid],[self.psid]]))-self.ki_lat*self.Int_psi + lat_error
        left_force = (F+Tau/d)/2.0
        right_force = (F-Tau/d)/2.0


        ##################################

        # Scale Output
        l_out = left_force/km
        if(l_out < 0):
            l_out = 0
        elif(l_out > .7):
            l_out = .7

        r_out = right_force/km
        if(r_out < 0):
            r_out = 0
        elif(r_out > .7):
            r_out = .7
        # print(r_out)
        # print(l_out)

        # Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        command.left_motor = 0
        command.right_motor = 0
        self.command_pub_.publish(command)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass
