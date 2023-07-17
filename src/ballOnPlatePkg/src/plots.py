#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, signal
import matplotlib.pyplot as plt 
import numpy as np

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
    folder_path = os.path.join(current_dir, 'figures')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    figure_path = os.path.join(folder_path, f'{figure_name}.png')
    plt.tight_layout()
    plt.savefig(figure_path, dpi=200)
    plt.close()
def plotTwoData(data1, data2, data3, data4, timestamp0, timestamp1, title, xLabel, yLabel, figure_name, label1='Label1', label2='Label2', label3='Label3', label4='Label4', limit=False): 
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
    folder_path = os.path.join(current_dir, 'figures')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    figure_path = os.path.join(folder_path, f'{figure_name}.png')
    plt.tight_layout()
    plt.savefig(figure_path, dpi=200)
    plt.close()

def saveArray(data1, data2, timestamp, filename): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'savedArrays')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    array_path = os.path.join(folder_path, f'{filename}.npy')

    stacked_array = np.vstack((data1, data2, timestamp))
    np.save(array_path, stacked_array)

def loadArray(): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays'
    file_list = [file for file in os.listdir(current_dir) if file.endswith(".npy")]
    data_arrays = []
    for file in file_list:
        print(file)
        file_path = os.path.join(current_dir, file)
        loaded_data = np.load(file_path)
        data_arrays.append(loaded_data)
    
    
    plotTwoAxis(data_arrays[0][0], data_arrays[0][1], data_arrays[0][2], 'TouchScreenFilteredData', 'Time (s)', 'Coordinate Value', 'TouchScreen', 'touchscreenFiltered', 'X-Axis', 'Y-Axis')
    plotTwoData(data_arrays[6][0], data_arrays[6][1], data_arrays[1][0], data_arrays[1][1], data_arrays[6][2], data_arrays[1][2], 'TouchscreenData', 'Time (s)', 'Coordinate Value', 'Touchscreen vs Nengo Touchscreen', 'X-Axis Normal', 'Y-Axis Normal', 'X-Axis nengo', 'Y-Axis Nengo')






if __name__ == '__main__':
    loadArray()