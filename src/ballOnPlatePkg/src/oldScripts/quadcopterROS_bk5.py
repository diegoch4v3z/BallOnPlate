# Diego Chavez
# Omar Garcia
# New Mexico State University

import numpy as np
import ctypes
import math
import sys
import time
import rospy
from scipy import signal as sig
import rostopic

#import run_model
#from run_model import QUADController
#from run_model import QUADController.QUADController.simple_vertical()

# XPlane_connect file responsible for the communication TO XPlane for sending the Motor Throttles
import Control_utlis as utlis

from nav_msgs.msg import Odometry
import xplane_ros.msg as xplane_msgs

sim_dt = 0.01
dt = 0.001

class Quadcopter(object):
    def __init__(self, target_func):
        rospy.init_node('xplane_subs', anonymous=True)

        self.reference_time = time.time()
        self.last_execution_time = time.time()


        self.target_func = target_func
        self.numtaps = 15
        #f = 0.0001
        f = 0.000001
        self.filter_coeff = sig.firwin(self.numtaps, f)
        self.input_buffer_p = []
        self.input_buffer_q = []
        self.input_buffer_phi = []
        self.input_buffer_theta = []
        self.pos_err = [0, 0, 0]
        self.lin = [0, 0, 0]
        self.ori_err = [0, 0, 0]
        self.ang = [0, 0, 0]
        self.count = 0
        #self.xplane_ori = [0, 0, 0]


        self.xplane_pos = [0, 0, 0]
        self.xplane_desired_pos, self.xplane_desired_ori = self.reference()


        #self.xplane_desired_pos = [0, 0, 0]
        #self.xplane_desired_ori = [0, 0, 0]
        self.lin_vel_err = [0,0,0]
        self.xplane_lin_vel = [0, 0, 0]
        self.xplane_ang_vel = [0,0,0]
        self.filtered_data_p = [0]
        self.filtered_data_q = [0]
        self.normalized_ez = [0]
        self.psi_deseada = 251.0
        self.is_yaw_set = False
        self.phi = 0
        self.theta = 0
        #self.psi = 0



        rospy.Subscriber("/xplane/flightmodel/global_state", xplane_msgs.GlobalState, self.callback)
        rospy.Subscriber("/xplane/flightmodel/odom", Odometry, self.odomcallback)


    def callback(self, data):
        self.phi = data.roll  * (3.14 / 180)
        self.theta = data.pitch  * (3.14 / 180)
        self.psi = data.heading  * (3.14 / 180)
        if not self.is_yaw_set:  # Bandera para guardar el angulo de yaw inicial
            self.yaw_init = data.heading * (3.14 / 180)
            self.is_yaw_set = True

        self.xplane_ori = [0, 0, self.yaw_init]
    def odomcallback(self, data):

        # XPlane Position
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

        #XPlane Linear Velocity
        self.vx = data.twist.twist.linear.x
        self.vy = data.twist.twist.linear.y
        self.vz = data.twist.twist.linear.z

        #XPlane Angular Velocity
        self.p = data.twist.twist.angular.x
        self.q = data.twist.twist.angular.y
        self.r = data.twist.twist.angular.z

        self.references = self.reference()


    def reference(self):
        current_time = time.time() - self.reference_time
        #print('current time el bueno', current_time)
        if current_time < 10:
            return [0, 0, -3*(current_time+0.3)], [0, 0, 0]
        else:
            return [0, 0, -30], [0, 0, 0]


    def get_target(self):
        #run = QUADController()
        #self.xplane_desired_pos, self.xplane_desired_ori = run.simple_vertical(100)
        #self.xplane_desired_ori = [0,0,self.yaw_init]
        #self.xplane_desired_pos = [0,0,-30]

        self.xplane_desired_pos, self.xplane_desired_ori = self.references
        #print('References en ek get target:', self.xplane_desired_pos)
        self.xplane_desired_ori = [0,0,self.yaw_init]


    def calculate_error(self):

        #Filtering p
        self.input_buffer_p.append(self.p)
        self.input_buffer_q.append(self.q)

        if len(self.input_buffer_p) >= self.numtaps:
            # Get the latest input samples from the buffer
            self.record_flag = True
            input_samples_p = self.input_buffer_p[-self.numtaps:]
            input_samples_q = self.input_buffer_q[-self.numtaps:]

            # Apply the filter operation
            self.filtered_data_p = sig.convolve(input_samples_p, self.filter_coeff, mode='valid')
            self.filtered_data_q = sig.convolve(input_samples_q, self.filter_coeff, mode='valid')
            #self.p = self.filtered_data_p   #Renaming filtered to be used


        self.xplane_ori = [self.phi, self.theta, self.psi]
        self.xplane_pos = [self.x, self.y, self.z]
        self.xplane_lin_vel = [self.vx, self.vy, self.vz]
        self.xplane_ang_vel = [self.p, self.q, self.r]
        self.ori_err = [self.xplane_ori[0] - self.xplane_desired_ori[0],
                        self.xplane_ori[1] - self.xplane_desired_ori[1],
                        self.xplane_ori[2] - self.xplane_desired_ori[2]]

        errorz = self.xplane_pos[2] - self.xplane_desired_pos[2]
        N = 200
        self.normalized_ez = errorz / N

        self.pos_err = [self.xplane_pos[0] - self.xplane_desired_pos[0],
                        self.xplane_pos[1] - self.xplane_desired_pos[1],
                        self.xplane_pos[2] - self.xplane_desired_pos[2]]

        '''self.pos_err = [self.xplane_pos[0] - self.xplane_desired_pos[0],
                        self.xplane_pos[1] - self.xplane_desired_pos[1],
                        self.normalized_ez]'''

        self.lin_vel_err = [self.xplane_lin_vel[0] - 0,
                            self.xplane_lin_vel[1] - 0,
                            self.xplane_lin_vel[2] - 0]




    def send_motor_commands(self, values):
        self.Throttle1 = values[0]
        self.Throttle2 = values[1]
        self.Throttle3 = values[2]
        self.Throttle4 = values[3]
        # Sending the motor Commands to XPlane using XPlane_Connect
        with utlis.XPlaneConnect() as client:
            self.data = [\
                [25, self.Throttle1, self.Throttle2, self.Throttle3, self.Throttle4, -998, -998, -998, -998], \
                [8, -998, -998, -998, -998, -998, -998, -998, -998], \
                ]
            #client.sendDATA(self.data)

    def handle_input(self, values):
        # Send motor commands to XPlane
        self.send_motor_commands(values)  # Remember this script handles the whole communication from and to XPlane. That's why it sends the Motors also.

        # Retrieve desired location
        self.get_target()

        # Calculate state error
        self.calculate_error()

        # Calculate rate of publication
        current_time = time.time()





    def get_state(self):
        '''return [self.xplane_pos, self.xplane_ori,
                self.xplane_lin_vel, self.xplane_ang_vel,
                self.xplane_desired_pos, self.xplane_desired_ori]'''
        '''return [self.pos_err[0], self.pos_err[1], self.pos_err[2],
                self.lin_vel_err[0], self.lin_vel_err[1], self.lin_vel_err[2],
                self.xplane_ori[0], self.xplane_ori[1], self.xplane_ori[2],
                self.xplane_ang_vel[0], self.xplane_ang_vel[1], self.xplane_ang_vel[2],self.filtered_data_p,
                ]'''
        return [self.xplane_desired_pos[2]
                ]


    # Error de posicion, add to the PDx,y in the diagram, eta , TODO Filtrar orientacion, Velocidad Angular
    def handle_output(self):
        matrix = np.array([[-np.sin(self.theta), 0, 1],
                           [np.cos(self.theta) * np.sin(self.phi), np.cos(self.phi), 0],
                           [np.cos(self.theta) * np.cos(self.phi), -np.sin(self.phi), 0]])
        return [self.pos_err[0], self.pos_err[1], self.pos_err[2],
                self.lin_vel_err[0], self.lin_vel_err[1], self.lin_vel_err[2],
                self.xplane_ori[0], self.xplane_ori[1], self.xplane_ori[2],
                self.xplane_ang_vel[0], self.xplane_ang_vel[1], self.xplane_ang_vel[2],
                self.xplane_desired_ori[-1], self.xplane_pos[2], self.xplane_desired_pos[2]]#Last 2: z and z_d
    def __call__(self, t, values):
        # TODO Check the frequency of xplane and nengo

        #self.references = self.reference()
        self.count += 1
        if self.count == 17:
            self.count = 0
            #self.references = self.reference()
            #print("references llamadas en el call", self.references)
            self.handle_input(values)

        return self.handle_output()







