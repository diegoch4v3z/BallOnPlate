#!/usr/bin/python

# Diego Chavez
# Omar Garcia
# New Mexico State University

import sys
import time
import nengo
import subprocess
import os
import signal
import cPickle as pickle
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
import os

import rospy
import matplotlib.pyplot as plt
import plotter as pltr

import quadcopterROS as QUAD
from nav_msgs.msg import Odometry

#import PID_Nengo as Model

dt = 0.001


class QUADController():

    #def signal_handler(signal, frame):
    print('Nengo Path:', nengo.__path__)

    def stop_other_code(self):
        # Killz the simulation of the conventionl PID when launched from the launch file for data comparisson
        process_name = "Control_QUAD_ROS_PID.py"
        os.system("pkill -f {}".format(process_name))

    def send_interrupt_to_other_code(self):
        # Assuming the other code's process name is "other_code.py"
        process_name = "Control_QUAD_ROS_PID.py"
        os.system("pkill -SIGINT -f {}".format(process_name))



    '''def simple_vertical(self):
        return [0, 0, -300.0], [0, 0, 0]'''

    def simple_vertical(self):
        current_time = time.time() - self.reference_time
        #print('current time', current_time)
        if current_time < 10:
            return [0, 0, -current_time], [0, 0, 0]
        else:
            return [0, 0, -30], [0, 0, 0]

    def __init__(self):
            self.is_time_set = False
            times = [0]

            if not self.is_time_set:                # Bandera para guardar el angulo de yaw inicial
                self.time_init = time.time()
                self.is_time_set = True

            #setting time to zero for height reference
            self.reference_time = time.time()
            self.zero_time = time.time() - self.reference_time
            #print("Elapsed time: ", self.zero_time)

            num_steps = 30000
            self.target_func = self.simple_vertical()
            model_name = 'PID_Nengo'
            exec("import %s as Model" % model_name)
            m = Model.Model(self.target_func)
            model = m.get_model()

            sim = nengo.Simulator(model, dt=dt)
            probesStateNode, probesStateEnsemble, probeDenormEz, probeEeOri, probeAngleVelInerDes, probe_pqrDesired, probes_posPID = m.get_probe() #TODO PROBES

            #print("running simulator...")

            xplane_state_data = []
            count = 0
            count_t = 0

            for i in range(num_steps):
                sim.step()
                self.target_func = self.simple_vertical()

                self.actual_time = time.time() - self.time_init
                times.append(self.actual_time)
                count += 1
                # TODO Revisar si el contador funciona
                if count == 17:
                    count = 0
                    count_t += 1
                    t = sim.trange()
                    dataProbesStateNode = sim.data[probesStateNode]
                    dataProbesStateEnsemble = sim.data[probesStateEnsemble]#Probing from nengo Node
                    #dataProbesErrorz = sim.data[probesErrorz]
                    dataProbesDenormErrorz = sim.data[probeDenormEz]
                    dataProbeEOri = sim.data[probeEeOri]
                    data_posPID = sim.data[probes_posPID]
                    #data_eOri = sim.data[probes_eOri]
                    #data_angleVelIner = sim.data[probe_angleVelIner]
                    dataProbeAngleVelInerDes = sim.data[probeAngleVelInerDes]
                    dataprobe_pqrDesired = sim.data[probe_pqrDesired]

                xplane_state_data.append(m.get_copter().get_state())   #Probing from Xplane

            timeLength = 29988

            pltr.plot_3(times[:timeLength], dataProbesStateNode[:,0], dataProbesStateNode[:,1], dataProbesStateNode[:,2], 'Error x', 'Meters (m)', 'Error y', 'Meters(m)', 'Error z', 'Meters (m)', 'Nengo Node Position Errors')
            pltr.plot_3(times[:timeLength], dataProbesStateNode[:,3], dataProbesStateNode[:,4], dataProbesStateNode[:,5], 'Error velocity x', 'Velocity (m/s)', 'Error velocity y', 'Velocity (m/s)', 'Error velocity z', 'Velocity (m/s)', 'Nengo Node Velocity Errors')
            pltr.plot_3(times[:timeLength], dataProbesStateNode[:,6], dataProbesStateNode[:,7], dataProbesStateNode[:,8], 'Phi', 'Radians (rad)', 'Theta', 'Radians (rad)', 'Psi', 'Radians (rad)', 'Nengo Node Euler Angles')
            pltr.plot_3(times[:timeLength], dataProbesStateNode[:,9], dataProbesStateNode[:,10], dataProbesStateNode[:,11], 'p','Angular Velocity (rad/s)', 'q', 'Angular Velocity (rad/s)', 'r', 'Angular Velocity (rad/s)', 'Nengo Node Angular Velocity Body Frame')
            pltr.plot_1(times[:timeLength], dataProbesStateNode[:,12], 'Desired Psi', 'radians (rad)', 'Nengo Node Desired Psi')

            pltr.plot_3(times[:timeLength], dataProbesStateEnsemble[:, 0], dataProbesStateEnsemble[:, 1], dataProbesStateEnsemble[:, 2], 'Error x', 'Meters (m)', 'Error y', 'Meters(m)', 'Error z', 'Meters (m)', 'Nengo Ensemble Position Errors')
            pltr.plot_3(times[:timeLength], dataProbesStateEnsemble[:, 3], dataProbesStateEnsemble[:, 4], dataProbesStateEnsemble[:, 5], 'Error velocity x', 'Velocity (m/s)', 'Error velocity y', 'Velocity (m/s)', 'Error velocity z', 'Velocity (m/s)', 'Nengo Ensemble Velocity Errors')
            pltr.plot_1(times[:timeLength], dataProbesStateEnsemble[:, 6], 'Integral Error z', 'Meters (m)', 'Nengo Ensemble Integral Error z')
            pltr.plot_3(times[:timeLength], dataProbesStateEnsemble[:, 7], dataProbesStateEnsemble[:, 8], dataProbesStateEnsemble[:, 9], 'Phi', 'Radians (rad)', 'Theta', 'Radians (rad)', 'Psi', 'Radians (rad)', 'Nengo Ensemble Euler Angles')
            pltr.plot_3(times[:timeLength], dataProbesStateEnsemble[:, 10], dataProbesStateEnsemble[:, 11], dataProbesStateEnsemble[:, 12], 'p', 'Angular Velocity (rad/s)', 'q', 'Angular Velocity (rad/s)', 'r', 'Angular Velocity (rad/s)', 'Nengo Ensemble Angular Velocity Body Frame')
            pltr.plot_1(times[:timeLength], dataProbesStateEnsemble[:, 13], 'Desired Psi', 'radians (rad)', 'Nengo Ensemble Desired Psi')

            pltr.plot_2_in_1(times[:timeLength], dataProbesStateEnsemble[:, 14], dataProbesStateEnsemble[:, 15], 'z', 'm', 'z_deseada', 'm', 'Altura z en Nengo')

            pltr.plot_1(times[:timeLength], dataProbesDenormErrorz, 'Integral Ez', 'm', 'Nengo Calculated Denormalized Integral of ez')
            pltr.plot_3(times[:timeLength], dataProbeEOri[:, 0], dataProbeEOri[:, 1], dataProbeEOri[:,2],'Error phi','rad','Error theta', 'rad', 'Error psi','rad', 'Nengo Orientation Errors')
            pltr.plot_3(times[:timeLength], dataProbeAngleVelInerDes[:,0], dataProbeAngleVelInerDes[:,1], dataProbeAngleVelInerDes[:,2], 'Psi Dot Desired', 'rad/s', 'Theta Dot Desired', 'rad/s','Phi Dot Desired', 'rad/s',  'Nengo Desired Angular Velocities Inertial Frame')
            pltr.plot_3(times[:timeLength],dataprobe_pqrDesired[:,0], dataprobe_pqrDesired[:,1], dataprobe_pqrDesired[:,2], 'p desired', 'rad/s', 'q desired', 'rad/s', 'r desired', 'rad/s', 'Nengo Desired Angular Velocities Body Frame')


            pltr.plot_3(times[:timeLength], data_posPID[:,0], data_posPID[:,1], data_posPID[:,2], 'PDy', 'PDy units', 'PDx', 'PDx units', 'PIDz', 'PIDz units', 'Nengo PID Position')
            # pltr.plot_3(times[:14989], data_eOri[:,0], data_eOri[:,1], data_eOri[:,2], 'error phi', 'rad?', 'error theta', 'rad?', 'error psi', 'rad?', 'Orientation errors Nengo')
            # pltr.plot_3(times[:14989], data_angleVelIner[:,0], data_angleVelIner[:,1], data_angleVelIner[:,2], 'desired_Psi_Dot', 'rad/s', 'desired_Theta_Dot', 'rad/s', 'desired_Phi_Dot', 'rad/s', 'Desired Angular Velocities Nengo')
            # pltr.plot_3(times[:14989], data_p_nengo[:,7], data_p_nengo[:,8], data_p_nengo[:,9], 'phi', 'rad', 'Theta', 'rad', 'Psi', 'rad', 'Euler Angles Nengo')
            # pltr.plot_2(times[:14989], data_p_nengo[:,2], data_ez_nengo, 'Error en Z', 'units', 'Integral en z', 'units', 'Integral y error en z NENGO')

            #Plotting
            refer_z = xplane_state_data

            refer_z = np.asarray(refer_z[0:len(t)])
            #norm_ez = norm_ez[:, np.newaxis]

            #pltr.plot_1(t[12:], refer_z[12:], 'e_z', 'rad/s', 'referencia en z Nengo')
            #pltr.plot_1(times[:14989], refer_z, 'e_z', 'rad/s', 'referencia en z Nengo nuevo')

            #pltr.plot_1(times[:14989], data_ez_nengo, 'integral ez norm en nengo', 'm', 'integral ez norm en nengo con nuevo vector times')


            self.send_interrupt_to_other_code()


def main():
  QUADcon = QUADController()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    #plt.close()
    pass



