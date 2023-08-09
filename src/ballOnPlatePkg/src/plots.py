#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, signal
import matplotlib.pyplot as plt 
import numpy as np
from datetime import datetime

def plotTwoAxis(data1, data2, timestamp, title, xLabel, yLabel, figure_name, label1='Label1', label2='Label2', limit=False): 
    plt.figure()
    plt.plot(timestamp, data1, label=label1)
    plt.plot(timestamp, data2, label=label2)
    plt.grid()
    plt.title(title)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    if limit: 
        plt.ylim([-1, 1])
    plt.legend()
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'plots')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    figure_path = os.path.join(folder_path, f'{figure_name}.png')
    plt.tight_layout()
    plt.savefig(figure_path, dpi=300)
    plt.close()
def plotTouchScreenData(dataX0, dataX1, timestamp0, timestamp1, title, xLabel, yLabel, figure_name, label1='Label1', label2='Label2', lim=False): 
    plt.figure()
    plt.plot(timestamp0, dataX0, label=label1)
    plt.plot(timestamp1, dataX1, label=label2)
    plt.grid()
    plt.title(title)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    if lim: 
        plt.ylim([-3, 3])
    plt.legend()
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'plots')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    figure_path = os.path.join(folder_path, f'{figure_name}.png')
    plt.tight_layout()
    plt.savefig(figure_path, dpi=300)
    plt.close()
def plotTwoData(data1, data2, data3, data4, timestamp0, timestamp1, title, xLabel, yLabel, figure_name, label1='Label1', label2='Label2', label3='Label3', label4='Label4', limit=True): 
    plt.figure()
    plt.plot(timestamp0, data1, label=label1)
    plt.plot(timestamp0, data2, label=label2)
    plt.plot(timestamp1, data3, label=label3)
    plt.plot(timestamp1, data4, label=label4)
    plt.grid()
    plt.title(title)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    if limit: 
        plt.ylim([-1, 1])
    plt.legend()
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'plots')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    figure_path = os.path.join(folder_path, f'{figure_name}.png')
    plt.tight_layout()
    plt.savefig(figure_path, dpi=200)
    plt.close()
def plotOneData(data0, timestamp0, title, xLabel, yLabel, figureName, label1, limit = False, lim=[0,0]):
    plt.figure()
    plt.plot(timestamp0, data0, label=label1)
    plt.grid()
    plt.title(title)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    plt.legend()
    if limit: 
        plt.ylim(lim)
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'plots')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    figure_path = os.path.join(folder_path, f'{figureName}.png')
    plt.tight_layout()
    plt.savefig(figure_path, dpi=300)
    plt.close()

def saveArray(data1, data2, timestamp, filename): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'savedArrays')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    array_path = os.path.join(folder_path, f'{filename}.npy')

    stacked_array = np.vstack((data1, data2, timestamp))
    np.save(array_path, stacked_array)
def saveArray1(data1, timestamp, filename): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'savedArrays')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    array_path = os.path.join(folder_path, f'{filename}.npy')

    stacked_array = np.vstack((data1, timestamp))
    np.save(array_path, stacked_array)

def loadArray(): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays'
    file_list = [file for file in os.listdir(current_dir) if file.endswith(".npy")]
    data_arrays = []
    for file in file_list:
        print(file)
        file_path = os.path.join(current_dir, file)
        loaded_data = np.load(file_path, allow_pickle=True)
        data_arrays.append(loaded_data)
    
    
   

    # 0 derivativeXNengo.npy
    # 1 errorNengoY.npy
    # 2 errorNengoX.npy
    # 3 touchScreenReadingNengo.npy
    # 4 touchScreenReadingRaw.npy
    # 5 setPointEnsembleNengo.npy
    # 6 controlNengo.npy
    # 7 derivativePD.npy
    # 8 derivativeYNengo.npy
    # 9 controlPD.npy
    # 10 errorPD.npy
    # 11 touchScreenReadingPD.npy

    plotOneData(data_arrays[0][0], data_arrays[0][1], 'Derivative X-Axis Nengo', 'Time (s)', 'Value', 'derivativeXNengo', 'Derivative X')
    plotOneData(data_arrays[8][0], data_arrays[8][1], 'Derivative Y-Axis Nengo', 'Time (s)', 'Value', 'derivativeYNengo', 'Derivative Y')
    plotOneData(data_arrays[1][0], data_arrays[1][1], 'Error Nengo Y', 'Time (s)', 'Value', 'errorNengoY', 'Y')
    plotOneData(data_arrays[2][0], data_arrays[2][1], 'Error Nengo X', 'Time (s)', 'Value', 'errorNengoX', 'X')
    plotTouchScreenData(data_arrays[4][0], data_arrays[3][0], data_arrays[4][2], data_arrays[3][2], 'Raw vs Nengo Touchscreen', 'Time (s)', 'Position', 'RawVsNengoTouchscreenX', 'X-Axis Raw', 'X-Axis Nengo')
    plotTouchScreenData(data_arrays[4][0], data_arrays[11][0], data_arrays[4][2], data_arrays[11][2], 'Raw vs PD Touchscreen', 'Time (s)', 'Position', 'RawVsPDTouchscreenX', 'X-Axis Raw', 'X-Axis PD')
    plotTouchScreenData(data_arrays[3][0], data_arrays[11][0], data_arrays[3][2], data_arrays[11][2], 'Nengo vs PD Touchscreen', 'Time (s)', 'Position', 'PDVsNengoTouchscreenX', 'X-Axis Nengo', 'X-Axis PD')
    
    plotTouchScreenData(data_arrays[4][1], data_arrays[3][1], data_arrays[4][2], data_arrays[3][2], 'Raw vs Nengo Touchscreen', 'Time (s)', 'Position', 'RawVsNengoTouchscreenY', 'Y-Axis Raw', 'Y-Axis Nengo')
    plotTouchScreenData(data_arrays[4][1], data_arrays[11][1], data_arrays[4][2], data_arrays[11][2], 'Raw vs PD Touchscreen', 'Time (s)', 'Position', 'RawVsPDTouchscreenY', 'Y-Axis Raw', 'Y-Axis PD')
    plotTouchScreenData(data_arrays[3][1], data_arrays[11][1], data_arrays[3][2], data_arrays[11][2], 'Nengo vs PD Touchscreen', 'Time (s)', 'Position', 'PDVsNengoTouchscreenY', 'Y-Axis Nengo', 'Y-Axis PD')
#6 nengo
#7 PD
    plotTouchScreenData(data_arrays[6][0], data_arrays[9][0], data_arrays[6][2], data_arrays[9][2], 'Nengo vs PD Control X', 'Time (s)', 'Value', 'PDVSNengoControlX', 'Control Nengo', 'Control PD', lim=True)
    plotTouchScreenData(data_arrays[6][1], data_arrays[9][1], data_arrays[6][2], data_arrays[9][2], 'Nengo vs PD Control Y', 'Time (s)', 'Value', 'PDVSNengoControlY', 'Control Nengo', 'Control PD', lim=True)

    plotTouchScreenData(data_arrays[4][2], data_arrays[3][2], np.linspace(0, len(data_arrays[4][2]), len(data_arrays[4][2])), np.linspace(0, len(data_arrays[3][2]), len(data_arrays[3][2])), 'TouchScreenTime vs Nengo Time', 'Index', 'Time', 'TouchScreenTimeVsNengoTime', 'TouchScreen Time', 'Nengo Time', lim=False)
    plotOneData(data_arrays[7][0], data_arrays[7][2], 'Derivative X PD', 'Time (s)', 'Value', 'derivativeXPD', 'Derivative X PD', limit = True, lim=[-1,1])
    plotOneData(data_arrays[7][1], data_arrays[7][2], 'Derivative Y PD', 'Time (s)', 'Value', 'derivativeYPD', 'Derivative Y PD', limit = True, lim=[-1,1])
    plotOneData(data_arrays[10][0], data_arrays[10][2], 'Error X PD', 'Time (s)', 'Value', 'errorXPD', 'Error X PD')
    plotOneData(data_arrays[10][1], data_arrays[10][2], 'Error Y PD', 'Time (s)', 'Value', 'errorYPD', 'Error Y PD')



if __name__ == '__main__':
    loadArray()