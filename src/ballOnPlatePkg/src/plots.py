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
def plotOneData(data0, timestamp0, title, xLabel, yLabel, figureName, label1):
    plt.figure()
    plt.plot(timestamp0, data0, label=label1)
    plt.grid()
    plt.title(title)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    plt.legend()
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
    
    
    # 

    plotOneData(data_arrays[0][0], data_arrays[0][1], 'Derivative X-Axis Nengo', 'Time (s)', 'Value', 'derivativeXNengo', 'Derivative X')
    plotOneData(data_arrays[1][0], data_arrays[1][1], 'Derivative Y-Axis Nengo', 'Time (s)', 'Value', 'derivativeYNengo', 'Derivative Y')
    plotTwoAxis(data_arrays[2][0], data_arrays[2][1], data_arrays[2][2], 'Error Nengo', 'Time (s)', 'Value', 'errorNengo', 'X', 'Y')
    plotTouchScreenData(data_arrays[4][0], data_arrays[3][0], data_arrays[4][2], data_arrays[3][2], 'Raw vs Nengo Touchscreen', 'Time (s)', 'Position', 'RawVsNengoTouchscreenX', 'X-Axis Raw', 'X-Axis Nengo')
    plotTouchScreenData(data_arrays[4][0], data_arrays[8][0], data_arrays[4][2], data_arrays[8][2], 'Raw vs PD Touchscreen', 'Time (s)', 'Position', 'RawVsPDTouchscreenX', 'X-Axis Raw', 'X-Axis PD')
    plotTouchScreenData(data_arrays[3][0], data_arrays[8][0], data_arrays[3][2], data_arrays[8][2], 'Nengo vs PD Touchscreen', 'Time (s)', 'Position', 'PDVsNengoTouchscreenX', 'X-Axis Nengo', 'X-Axis PD')
    
    plotTouchScreenData(data_arrays[4][1], data_arrays[3][1], data_arrays[4][2], data_arrays[3][2], 'Raw vs Nengo Touchscreen', 'Time (s)', 'Position', 'RawVsNengoTouchscreenY', 'Y-Axis Raw', 'Y-Axis Nengo')
    plotTouchScreenData(data_arrays[4][1], data_arrays[8][1], data_arrays[4][2], data_arrays[8][2], 'Raw vs PD Touchscreen', 'Time (s)', 'Position', 'RawVsPDTouchscreenY', 'Y-Axis Raw', 'Y-Axis PD')
    plotTouchScreenData(data_arrays[3][1], data_arrays[8][1], data_arrays[3][2], data_arrays[8][2], 'Nengo vs PD Touchscreen', 'Time (s)', 'Position', 'PDVsNengoTouchscreenY', 'Y-Axis Nengo', 'Y-Axis PD')
#6 nengo
#7 PD
    plotTouchScreenData(data_arrays[6][0], data_arrays[7][0], data_arrays[6][2], data_arrays[7][2], 'PD vs Nengo Control X', 'Time (s)', 'Value', 'PDVSNengoControlX', 'Control Nengo', 'Control PD', lim=True)
    plotTouchScreenData(data_arrays[6][1], data_arrays[7][1], data_arrays[6][2], data_arrays[7][2], 'PD vs Nengo Control Y', 'Time (s)', 'Value', 'PDVSNengoControlY', 'Control Nengo', 'Control PD', lim=True)
    # PLOT the raw touchscreen data (NENGO, PD)
    # PLOT the control data (Nengo, PD)
    # plotTwoData(data_arrays[8][0], data_arrays[8][1], data_arrays[9][0], data_arrays[9][1], data_arrays[8][2], data_arrays[9][2], 'Nengo Touchscreen Reading vs PD Touchscreen Reading', 'Time (s)', 'Position', 'NengoVSPDtouchscreenReading', 'X-Axis Touchscreen Nengo', 'Y-Axis Touchscreen Nengo', 'X-Axis Touchscreen PD Conventional', 'Y-Axis Touchscreen PD Conventional')
    # plotTouchScreenData(data_arrays[9][0], data_arrays[8][0], data_arrays[9][2], data_arrays[8][2], 'Nengo Touchscreen Reading vs PD Touchscreen Reading', 'Time (s)', 'Position', 'touchScreenReadingNengoPD X-Axis', 'Touchscreen reading PD X-Axis', 'Touchscreen reading Nengo - X Axis')
    # plotTouchScreenData(data_arrays[9][1], data_arrays[8][1], data_arrays[9][2], data_arrays[8][2], 'Nengo Touchscreen Reading vs PD Touchscreen Reading', 'Time (s)', 'Position', 'touchScreenReadingNengoPD Y-Axis', 'Touchscreen reading PD Y-Axis', 'Touchscreen reading Nengo - Y Axis')
    # plotTouchScreenData(data_arrays[0][1], data_arrays[8][1], data_arrays[0][2], data_arrays[8][2], 'Nengo Touchscreen Reading vs PD Touchscreen Filtered Reading', 'Time (s)', 'Position', 'touchScreenReadingNengoPD Y-Axis filtered', 'Touchscreen filtered reading PD Y-Axis', 'Touchscreen reading Nengo - Y Axis')
    
    #plotTwoData(data_arrays[6][0], data_arrays[6][1], data_arrays[1][0], data_arrays[1][1], data_arrays[6][2], data_arrays[1][2], 'TouchscreenData dt=0.001', 'Time (s)', 'Coordinate Value', 'Touchscreen vs Nengo Touchscreen', 'X-Axis Touchscreen Nengo', 'Y-Axis Touchscreen Nengo', 'X-Axis Touchscreen PD Conventional', 'Y-Axis Touchscreen PD Conventional')
    #plotTwoAxis(data_arrays[5][0], data_arrays[5][1], data_arrays[5][2], 'ErrorNengo', 'Time (s)', 'Coordinate Value', 'ErrorNengo', 'ErrorNengo', 'X-Axis', 'Y-Axis')
    #plotTwoAxis(data_arrays[])

    # saveArray(dataProbe0[:, 0], dataProbe0[:, 1], self.timeSeries, 'touchScreenNengoData')
    #     saveArray(dataProbe1[:, 0], dataProbe1[:, 1], self.timeSeries, 'SetPointNengoData')
    #     saveArray(dataProbe2[:, 0], dataProbe2[:, 1], self.timeSeries, 'ErrorNengoData')
    #     saveArray(dataProbe3[:, 0], dataProbe3[:, 1], self.timeSeries, 'DerivativeXNengoData')
    #     saveArray(dataProbe4[:, 0], dataProbe4[:, 1], self.timeSeries, 'DerivativeYNengoData')
    #     saveArray(dataProbe5[:, 0], dataProbe5[:, 1], self.timeSeries, 'ControlSignalNengoData')



if __name__ == '__main__':
    loadArray()