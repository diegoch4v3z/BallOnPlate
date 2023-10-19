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
    folder_path = os.path.join(current_dir, 'measurements')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    array_path = os.path.join(folder_path, f'{filename}.npy')
    arrayCSV_path = os.path.join(folder_path, f'{filename}.csv')

    stacked_array = np.vstack((data1, data2, timestamp))
    np.save(array_path, stacked_array)
    np.savetxt(arrayCSV_path, stacked_array, delimiter=',' )
def saveArray1(data1, timestamp, filename): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'savedArrays')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    array_path = os.path.join(folder_path, f'{filename}.npy')

    stacked_array = np.vstack((data1, timestamp))
    np.save(array_path, stacked_array)

def saveArray2(data1, data2, timestamp, filename): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'savedArraysModified')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    array_path = os.path.join(folder_path, f'{filename}.npy')
    arrayCSV_path = os.path.join(folder_path, f'{filename}.csv')

    stacked_array = np.vstack((data1, data2, timestamp))
    np.save(array_path, stacked_array)
    np.savetxt(arrayCSV_path, stacked_array, delimiter=',' )

def saveArrayACC(data1, data2, timestamp, filename): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'ArraysACC')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    array_path = os.path.join(folder_path, f'{filename}.npy')
    stacked_array = np.vstack((data1, data2, timestamp))
    np.save(array_path, stacked_array)

def saveArrayLQR(data1, data2, timestamp, filename): 
    import os
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrayLQR')
    folder_path = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrayLQR'
    array_path = os.path.join(folder_path, f'{filename}.npy')
    arrayCSV_path = os.path.join(folder_path, f'{filename}.csv')
    stacked_array = np.vstack((data1, data2, timestamp))
    np.save(array_path, stacked_array)
    np.savetxt(arrayCSV_path, stacked_array, delimiter=',' )

def saveTimeArray(data1, dt, filename): 
    import os
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/tiempos')
    folder_path = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/tiempos'
    filename = filename + str(dt)
    array_path = os.path.join(folder_path, f'{filename}.npy')
    arrayCSV_path = os.path.join(folder_path, f'{filename}.csv')
    np.save(array_path, data1)
    np.savetxt(arrayCSV_path, data1, delimiter=',' )

def saveTimearrayLQG(data1, filename): 
    import os 
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification')
    folder_path = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification'
    array_path = os.path.join(folder_path, f'{filename}.npy') 
    arrayCSV_path = os.path.join(folder_path, f'{filename}.csv')
    np.save(array_path, data1)
    np.savetxt(arrayCSV_path, data1, delimiter=',')

def saveTimeArrayTouchscreen(data1, filename): 
    import os 
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification')
    folder_path = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification'
    array_path = os.path.join(folder_path, f'{filename}.npy') 
    arrayCSV_path = os.path.join(folder_path, f'{filename}.csv')
    np.save(array_path, data1)
    np.savetxt(arrayCSV_path, data1, delimiter=',')
def saveTimeArrayServos(data1, filename): 
    import os 
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification')
    folder_path = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification'
    array_path = os.path.join(folder_path, f'{filename}.npy') 
    arrayCSV_path = os.path.join(folder_path, f'{filename}.csv')
    np.save(array_path, data1)
    np.savetxt(arrayCSV_path, data1, delimiter=',')

def loadArray(): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays'
    file_list = [file for file in os.listdir(current_dir) if file.endswith(".npy")]
    data_arrays = []
    for file in file_list:
        print(current_dir)
        file_path = os.path.join(current_dir, file)
        loaded_data = np.load(file_path, allow_pickle=True)
        data_arrays.append(loaded_data)

def plotOneDataLQR(data0, timestamp0, title, xLabel, yLabel, figureName, label1, limit = False, lim=[0,0]):
    plt.figure()
    plt.plot(timestamp0, data0, label=label1)
    plt.grid()
    plt.title(title)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    plt.legend()
    if limit: 
        plt.ylim(lim)
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/plotsLQR'
    folder_path = os.path.join(current_dir, 'plots')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    figure_path = os.path.join(folder_path, f'{figureName}.png')
    plt.tight_layout()
    plt.savefig(figure_path, dpi=300)
    plt.close()

def plotLQR(): 
    controlSignalData = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrayLQR/controlSignalData.npy')
    fcxData = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrayLQR/fcxNengo.npy')
    fcyData = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrayLQR/fcyNengo.npy')
    #servoSignalData = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays/servoSignalData.npy')
    touchScreenReadingRawData = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrayLQR/touchScreenReadingRaw.npy')
    
    
    print(np.shape(fcxData[0][:]))
    plotOneDataLQR(fcxData[0][:], fcxData[2][:], 'FCX_Position', 'Time (s)', 'Value', 'FCX_Position', 'FCXNengo')
    plotOneDataLQR(fcxData[1][:], fcxData[2][:], 'FCX_Velocity', 'Time (s)', 'Value', 'FCX_Velocity', 'FCXNengo')
    plotOneDataLQR(fcyData[0][:], fcyData[2][:], 'FCY_Position', 'Time (s)', 'Value', 'FCY_Position', 'FCYNengo')
    plotOneDataLQR(fcyData[1][:], fcyData[2][:], 'FCY_Velocity', 'Time (s)', 'Value', 'FCY_Velocity', 'FCYNengo')
    plotOneDataLQR(controlSignalData[0][:], controlSignalData[2][:], 'controlSignalX', 'Time (s)', 'Value', 'controlSignalX', 'controlSignalX')
    plotOneDataLQR(controlSignalData[1][:], controlSignalData[2][:], 'controlSignalY', 'Time (s)', 'Value', 'controlSignalY', 'controlSignalY')
    plotOneDataLQR(touchScreenReadingRawData[0][:], touchScreenReadingRawData[2][:], 'RawSignalX', 'Time (s)', 'Value', 'RawSignalX', 'RawSignalX')
    plotOneDataLQR(touchScreenReadingRawData[1][:], touchScreenReadingRawData[2][:], 'RawSignalY', 'Time (s)', 'Value', 'RawSignalY', 'RawSignalY')

#     plotOneData(data_arrays[8][0], data_arrays[8][1], 'Derivative Y-Axis Nengo', 'Time (s)', 'Value', 'derivativeYNengo', 'Derivative Y')
#     plotOneData(data_arrays[1][0], data_arrays[1][1], 'Error Nengo Y', 'Time (s)', 'Value', 'errorNengoY', 'Y')
#     plotOneData(data_arrays[2][0], data_arrays[2][1], 'Error Nengo X', 'Time (s)', 'Value', 'errorNengoX', 'X')

    
def plotCCE(): 
    touschreenReadingRaw = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays/touchScreenReadingRaw.npy')
    touschreenReadingFiltered = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays/touchScreenReadingFiltered.npy')
    touchscreenNengo = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays/touchScreenReadingNengo.npy')
    setPointReading = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays/setPointEnsembleNengo.npy')
    controlSignal = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/savedArrays/controlNengo.npy')

    #  saveArray2(self.dataXPlot, self.dataYPlot, self.timeSeries, 'touchScreenReadingRaw')#, 'TouchScreen Reading', 'Time (s)', 'Coordinate Position', 'touchScreenData', 'X-Axis', 'Y-Axis', limit=True) 
    #  saveArray2(self.dataXPlotFiltered, self.dataYPlotFiltered, self.timeSeries[:len(self.dataXPlotFiltered)], 'touchScreenReadingFiltered')

    # Plotting
    figs, axs = plt.subplots(3,3, figsize=(6.67, 5.0))

    axs[0, 0].plot(touchscreenNengo[2][:], touchscreenNengo[0][:], color='black')
    axs[0, 0].plot(setPointReading[2,:], setPointReading[0,:], color='red')
    axs[0, 0].plot(controlSignal[2,:], controlSignal[0,:], color='blue')
    axs[0, 0].tick_params(axis='both', which='major', labelsize=6)
    axs[0, 0].get_xaxis().set_visible(False)
    axs[0, 0].set_ylim(-0.75, 0.75)
    axs[0, 0].set_yticks([-0.75, 0, 0.75])
    axs[0, 0].set_title('Subplot 1')

    axs[0, 1].plot()
    axs[0, 1].set_title('Subplot 1')
    axs[0, 1].set_ylim(-0.75, 0.75)
    axs[0, 1].set_yticks([-0.75, 0, 0.75])
    axs[0, 1].tick_params(axis='both', which='major', labelsize=6)

    axs[0, 2].plot()
    axs[0, 2].set_title('Subplot 1')
    axs[0, 2].set_ylim(-0.75, 0.75)
    axs[0, 2].set_yticks([-0.75, 0, 0.75])
    axs[0, 2].tick_params(axis='both', which='major', labelsize=6)

    axs[1, 0].plot()
    axs[1, 0].set_title('Subplot 1')
    axs[1, 0].tick_params(axis='both', which='major', labelsize=6)

    axs[1, 1].plot()
    axs[1, 1].set_title('Subplot 1')
    axs[1, 1].set_ylim(-0.75, 0.75)
    axs[1, 1].set_yticks([-0.75, 0, 0.75])
    axs[1, 1].tick_params(axis='both', which='major', labelsize=6)

    axs[1, 2].plot()
    axs[1, 2].set_title('Subplot 1')
    axs[1, 2].set_ylim(-0.75, 0.75)
    axs[1, 2].set_yticks([-0.75, 0, 0.75])
    axs[1, 2].tick_params(axis='both', which='major', labelsize=6)
   
    axs[2, 0].plot()
    axs[2, 0].set_title('Subplot 1')
    axs[2, 0].set_ylim(-0.75, 0.75)
    axs[2, 0].set_yticks([-0.75, 0, 0.75])
    axs[2, 0].tick_params(axis='both', which='major', labelsize=6)

    axs[2, 1].plot()
    axs[2, 1].set_title('Subplot 1')
    axs[2, 1].set_ylim(-0.75, 0.75)
    axs[2, 1].set_yticks([-0.75, 0, 0.75])
    axs[2, 1].tick_params(axis='both', which='major', labelsize=6)

    axs[2, 2].plot()
    axs[2, 2].set_title('Subplot 1')
    axs[2, 2].set_ylim(-0.75, 0.75)
    axs[2, 2].set_yticks([-0.75, 0, 0.75])
    axs[2, 2].tick_params(axis='both', which='major', labelsize=6)

    plt.subplots_adjust(wspace=0, hspace=0)
    plt.tight_layout()

    save_folder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/plots' 
    file_name = 'results.png' 
    dpi_value = 300  
    save_path = f"{save_folder}/{file_name}"
    plt.savefig(save_path, dpi=dpi_value)
 

#     plotOneData(data_arrays[0][0], data_arrays[0][1], 'Derivative X-Axis Nengo', 'Time (s)', 'Value', 'derivativeXNengo', 'Derivative X')
#     plotOneData(data_arrays[8][0], data_arrays[8][1], 'Derivative Y-Axis Nengo', 'Time (s)', 'Value', 'derivativeYNengo', 'Derivative Y')
#     plotOneData(data_arrays[1][0], data_arrays[1][1], 'Error Nengo Y', 'Time (s)', 'Value', 'errorNengoY', 'Y')
#     plotOneData(data_arrays[2][0], data_arrays[2][1], 'Error Nengo X', 'Time (s)', 'Value', 'errorNengoX', 'X')
#     plotTouchScreenData(data_arrays[4][0], data_arrays[3][0], data_arrays[4][2], data_arrays[3][2], 'Raw vs Nengo Touchscreen', 'Time (s)', 'Position', 'RawVsNengoTouchscreenX', 'X-Axis Raw', 'X-Axis Nengo')
#     plotTouchScreenData(data_arrays[4][0], data_arrays[11][0], data_arrays[4][2], data_arrays[11][2], 'Raw vs PD Touchscreen', 'Time (s)', 'Position', 'RawVsPDTouchscreenX', 'X-Axis Raw', 'X-Axis PD')
#     plotTouchScreenData(data_arrays[3][0], data_arrays[11][0], data_arrays[3][2], data_arrays[11][2], 'Nengo vs PD Touchscreen', 'Time (s)', 'Position', 'PDVsNengoTouchscreenX', 'X-Axis Nengo', 'X-Axis PD')
    
#     plotTouchScreenData(data_arrays[4][1], data_arrays[3][1], data_arrays[4][2], data_arrays[3][2], 'Raw vs Nengo Touchscreen', 'Time (s)', 'Position', 'RawVsNengoTouchscreenY', 'Y-Axis Raw', 'Y-Axis Nengo')
#     plotTouchScreenData(data_arrays[4][1], data_arrays[11][1], data_arrays[4][2], data_arrays[11][2], 'Raw vs PD Touchscreen', 'Time (s)', 'Position', 'RawVsPDTouchscreenY', 'Y-Axis Raw', 'Y-Axis PD')
#     plotTouchScreenData(data_arrays[3][1], data_arrays[11][1], data_arrays[3][2], data_arrays[11][2], 'Nengo vs PD Touchscreen', 'Time (s)', 'Position', 'PDVsNengoTouchscreenY', 'Y-Axis Nengo', 'Y-Axis PD')
# #6 nengo
# #7 PD
#     plotTouchScreenData(data_arrays[6][0], data_arrays[9][0], data_arrays[6][2], data_arrays[9][2], 'Nengo vs PD Control X', 'Time (s)', 'Value', 'PDVSNengoControlX', 'Control Nengo', 'Control PD', lim=True)
#     plotTouchScreenData(data_arrays[6][1], data_arrays[9][1], data_arrays[6][2], data_arrays[9][2], 'Nengo vs PD Control Y', 'Time (s)', 'Value', 'PDVSNengoControlY', 'Control Nengo', 'Control PD', lim=True)

#     plotTouchScreenData(data_arrays[4][2], data_arrays[3][2], np.linspace(0, len(data_arrays[4][2]), len(data_arrays[4][2])), np.linspace(0, len(data_arrays[3][2]), len(data_arrays[3][2])), 'TouchScreenTime vs Nengo Time', 'Index', 'Time', 'TouchScreenTimeVsNengoTime', 'TouchScreen Time', 'Nengo Time', lim=False)
#     plotOneData(data_arrays[7][0], data_arrays[7][2], 'Derivative X PD', 'Time (s)', 'Value', 'derivativeXPD', 'Derivative X PD', limit = True, lim=[-1,1])
#     plotOneData(data_arrays[7][1], data_arrays[7][2], 'Derivative Y PD', 'Time (s)', 'Value', 'derivativeYPD', 'Derivative Y PD', limit = True, lim=[-1,1])
#     plotOneData(data_arrays[10][0], data_arrays[10][2], 'Error X PD', 'Time (s)', 'Value', 'errorXPD', 'Error X PD')
#     plotOneData(data_arrays[10][1], data_arrays[10][2], 'Error Y PD', 'Time (s)', 'Value', 'errorYPD', 'Error Y PD')
#     plotOneData(data_arrays[5][0], data_arrays[5][2], 'SetPoint X', 'Time (s)', 'Value', 'setPointXNengo', 'Setpoint X Nengo')
#     plotOneData(data_arrays[5][1], data_arrays[5][2], 'SetPoint Y', 'Time (s)', 'Value', 'setPointYNengo', 'Setpoint Y Nengo')

def plotACC(): 
    import os
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC')
    control = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/controlNengo.npy')
    error = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/errorNengo.npy')
    setPoint = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/setPointEnsembleNengo.npy')
    touchReading = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/touchScreenReadingRaw.npy')
    touchReadingNengo = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/touchScreenReadingNengo.npy')
    disturbance = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/disturbanceData.npy')
    print('control: ', np.shape(control))
    print('error: ', np.shape(error))
    print('setPoint: ', np.shape(setPoint))
    print('touchReading: ', np.shape(touchReading))
    print('Disturbance', np.shape(disturbance))
    plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
    plt.rcParams['font.serif'] = 'DejaVu Serif'
    plt.rcParams['pdf.fonttype'] = 42


    fig, (ax1, ax2) = plt.subplots(2)
    
    ax1.plot(control[2][:], control[0][:], color='red', label='ux')
    ax1.set_ylabel('ux (rad)', color = 'black', fontsize=14)
    ax1.set_yticks([-0.4, -0.2, 0, 0.2, 0.4])
    
    ax3 = ax1.twinx()
    ax3.plot(touchReadingNengo[2][:] , touchReadingNengo[0][:] * 52.5 - 1, color='blue', label='x')
    ax3.plot(setPoint[2][:], setPoint[0][:], color='black', label='xd')
    ax3.set_ylabel('x, xd (mm)', fontsize=14)
    ax3.set_yticks([-20, -10, 0, 10, 20])
    ax1.set_xticks([0, 5, 10, 15, 20])
    ax3.set_xticks([0, 5, 10, 15, 20])

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax3.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax3.get_ylim()
    max_range = max(abs(y1_max), abs(y1_min), abs(y2_max), abs(y2_min))
    
    
    ax3.set_ylim(-20, 20)
    ax1.set_ylim(-0.50, 0.50) 
    ax3.set_xlim(0, 20)
    ax1.set_xlim(0, 20)
    ax1.set_xlabel('Time (s)', fontsize=14)
    ax3.set_xlabel('Time (s)', fontsize=14)

    ax2.plot(error[2][:], error[0][:]* 52.5 - 1 + 0.030*52.5, color='blue', label='error')
    disturbance[0][1301] = 0 
    disturbance[0][1051] = 0.25
    disturbance[0][3501] = 0 
    disturbance[0][3251] = 0.25

    ax2.plot(disturbance[2][:], disturbance[0][:]* 52.5  , color='red', label ='impulse')
    ax2.plot(setPoint[2][:], setPoint[0][:], color='black', label='xd')

    #axs[0, 0].plot(controlSignal[2,:], controlSignal[0,:], color='blue')
    #ax2.tick_params(axis='both', which='major', labelsize=6)
    #ax2.get_xaxis().set_visible(False)
    ax2.set_ylim(-20, 20)
    ax2.set_yticks([-20, -10, 0, 10, 20])
    ax2.set_xticks([0, 5, 10, 15, 20])
    ax2.set_ylabel('error (mm)', fontsize=14)
    ax2.set_xlabel('Time (s)', fontsize=14)

    ax2.legend()
    ax2.set_xlim(0, 20)
    #ax2.

    ax1.grid()
    ax2.grid()

    plt.subplots_adjust(wspace=0, hspace=0)
    plt.tight_layout()
    

    save_folder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC' 
    file_name = 'XaxisPlot.pdf' 
    dpi_value = 300  
    save_path = f"{save_folder}/{file_name}"
    plt.savefig(save_path, dpi=dpi_value)
 
def plotACCY():
    import os
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements')
    control = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/controlNengo.npy')
    error = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/errorNengo.npy')
    setPoint = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/setPointEnsembleNengo.npy')
    touchReading = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/touchScreenReadingRaw.npy')
    touchReadingNengo = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/touchScreenReadingNengo.npy')
    disturbance = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/disturbanceData.npy')
    print('control: ', np.shape(control))
    print('error: ', np.shape(error))
    print('setPoint: ', np.shape(setPoint))
    print('touchReading: ', np.shape(touchReading))
    print('Disturbance', np.shape(disturbance))
    plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
    plt.rcParams['font.serif'] = 'DejaVu Serif'
    plt.rcParams['pdf.fonttype'] = 42

    fig, (ax1, ax2) = plt.subplots(2)
    
    ax1.plot(control[2][:], control[1][:], color='red', label='uy')
    ax1.set_ylabel('uy (rad)', color = 'black', fontsize=14)
    ax1.set_yticks([-0.4, -0.2, 0, 0.2, 0.4], fontname='DejaVu Serif')
    ax1.set_xlabel('Time (s)', fontsize=14)
    
    ax3 = ax1.twinx()
    
    ax3.plot(touchReadingNengo[2][:] , touchReadingNengo[1][:] * 82.5 - 1, color='blue', label='y')
    ax3.plot(setPoint[2][:], setPoint[1][:], color='black', label='yd')
    ax3.set_ylabel('y, yd (mm)', fontsize=14)
    ax3.set_yticks([-20, -10, 0, 10, 20], fontname='DejaVu Serif')
    ax3.set_xticks([0, 5, 10, 15, 20], fontname='DejaVu Serif')
    ax1.set_xticks([0, 5, 10, 15, 20], fontname='DejaVu Serif')
    ax3.set_xlabel('Time (s)', fontsize=14)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax3.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax3.get_ylim()
    max_range = max(abs(y1_max), abs(y1_min), abs(y2_max), abs(y2_min))
    ax3.set_ylim(-20, 20)
    ax1.set_ylim(-0.50, 0.50)
    ax3.set_xlim(0, 20) 
    ax1.set_xlim(0, 20) 

    ax2.plot(error[2][:], error[1][:]* 82.5 - 1 + 0.015*82.5, color='blue', label='error')
    disturbance[0][1301] = 0 
    disturbance[0][1051] = 0.25
    disturbance[0][3501] = 0 
    disturbance[0][3251] = 0.25
    ax2.plot(disturbance[2][:], disturbance[0][:]*52.5 , color='red', label ='impulse')
    ax2.plot(setPoint[2][:], setPoint[1][:], color='black', label='yd')
    #axs[0, 0].plot(controlSignal[2,:], controlSignal[0,:], color='blue')
    #ax2.tick_params(axis='both', which='major', labelsize=6)
    #ax2.get_xaxis().set_visible(False)
    ax2.set_ylim(-20, 20)
    ax2.set_xlim(0, 20)
    ax2.set_yticks([-20, -10, 0, 10, 20], fontname='DejaVu Serif')
    ax2.set_xticks([0, 5, 10, 15, 20], fontname='DejaVu Serif')
    ax2.set_ylabel('error (mm)', fontsize=14)
    ax2.set_xlabel('Time (s)', fontsize=14)
    ax2.legend()
    #ax2.

    ax1.grid()
    ax2.grid()

    plt.subplots_adjust(wspace=0, hspace=0)
    plt.tight_layout()
    

    save_folder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC' 
    file_name = 'YaxisPlot.pdf' 
    dpi_value = 300  
    save_path = f"{save_folder}/{file_name}"
    plt.savefig(save_path, dpi=dpi_value)


def plotACC_PID_Y(): 
    import os
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC')
    control = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/control_PID.npy')
    error = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/error_PID.npy')
    setPoint = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/setPoint_PID.npy')
    touchReading = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/touchScreenReadingRaw.npy')
    touchReadingNengo = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/touchScreen_PID.npy')
    disturbance = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/disturbanceData.npy')
    print('control: ', np.shape(control))
    print('error: ', np.shape(error))
    print('setPoint: ', np.shape(setPoint))
    print('touchReading: ', np.shape(touchReading))
    print('Disturbance', np.shape(disturbance))
    plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
    plt.rcParams['font.serif'] = 'DejaVu Serif'
    plt.rcParams['pdf.fonttype'] = 42


    fig, (ax1, ax2) = plt.subplots(2)
    
    ax1.plot(control[2][:], control[1][:], color='red', label='uy')
    ax1.set_ylabel('uy (rad)', color = 'black', fontsize=14)
    ax1.set_yticks([-0.4, -0.2, 0, 0.2, 0.4])
    
    ax3 = ax1.twinx()
    ax3.plot(touchReadingNengo[2][:] , touchReadingNengo[1][:] * 52.5 - 0.0, color='blue', label='y')
    ax3.plot(setPoint[2][:], setPoint[1][:], color='black', label='yd')
    ax3.set_ylabel('y, yd (mm)', fontsize=14)
    ax3.set_yticks([-20, -10, 0, 10, 20])
    ax1.set_xticks([0, 5, 10, 15, 20])
    ax3.set_xticks([0, 5, 10, 15, 20])

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax3.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax3.get_ylim()
    max_range = max(abs(y1_max), abs(y1_min), abs(y2_max), abs(y2_min))
    
    
    ax3.set_ylim(-20, 20)
    ax1.set_ylim(-0.50, 0.50) 
    ax3.set_xlim(0, 20)
    ax1.set_xlim(0, 20)
    ax1.set_xlabel('Time (s)', fontsize=14)
    ax3.set_xlabel('Time (s)', fontsize=14)

    ax2.plot(error[2][:], error[1][:]* 52.5 - 1 + 0.02*52.5, color='blue', label='error')
    ax2.plot(disturbance[2][:], disturbance[1][:]* 52.5 , color='red', label ='impulse')
    ax2.plot(setPoint[2][:], setPoint[1][:], color='black', label='yd')
    #axs[0, 0].plot(controlSignal[2,:], controlSignal[0,:], color='blue')
    #ax2.tick_params(axis='both', which='major', labelsize=6)
    #ax2.get_xaxis().set_visible(False)
    ax2.set_ylim(-20, 20)
    ax2.set_yticks([-20, -10, 0, 10, 20])
    ax2.set_xticks([0, 5, 10, 15, 20])
    ax2.set_ylabel('error (mm)', fontsize=14)
    ax2.set_xlabel('Time (s)', fontsize=14)

    ax2.legend()
    ax2.set_xlim(0, 20)
    #ax2.

    ax1.grid()
    ax2.grid()

    plt.subplots_adjust(wspace=0, hspace=0)
    plt.tight_layout()
    

    save_folder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC' 
    file_name = 'YaxisPlot_PID.png' 
    dpi_value = 600  
    save_path = f"{save_folder}/{file_name}"
    plt.savefig(save_path, dpi=dpi_value)

def plotACC_PID(): 
    import os
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC')
    control = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/control_PID.npy')
    error = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/error_PID.npy')
    setPoint = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/setPoint_PID.npy')
    touchReading = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/touchScreenReadingRaw.npy')
    touchReadingNengo = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/touchScreen_PID.npy')
    disturbance = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC/disturbanceData.npy')
    print('control: ', np.shape(control))
    print('error: ', np.shape(error))
    print('setPoint: ', np.shape(setPoint))
    print('touchReading: ', np.shape(touchReading))
    print('Disturbance', np.shape(disturbance))
    plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
    plt.rcParams['font.serif'] = 'DejaVu Serif'
    plt.rcParams['pdf.fonttype'] = 42


    fig, (ax1, ax2) = plt.subplots(2)
    
    ax1.plot(control[2][:], control[0][:], color='red', label='ux')
    ax1.set_ylabel('ux (rad)', color = 'black', fontsize=14)
    ax1.set_yticks([-0.4, -0.2, 0, 0.2, 0.4])
    
    ax3 = ax1.twinx()
    ax3.plot(touchReadingNengo[2][:] , touchReadingNengo[0][:] * 52.5 - 1.0, color='blue', label='x')
    ax3.plot(setPoint[2][:], setPoint[0][:], color='black', label='xd')
    ax3.set_ylabel('x, xd (mm)', fontsize=14)
    ax3.set_yticks([-20, -10, 0, 10, 20])
    ax1.set_xticks([0, 5, 10, 15, 20])
    ax3.set_xticks([0, 5, 10, 15, 20])

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax3.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax3.get_ylim()
    max_range = max(abs(y1_max), abs(y1_min), abs(y2_max), abs(y2_min))
    
    
    ax3.set_ylim(-20, 20)
    ax1.set_ylim(-0.50, 0.50) 
    ax3.set_xlim(0, 20)
    ax1.set_xlim(0, 20)
    ax1.set_xlabel('Time (s)', fontsize=14)
    ax3.set_xlabel('Time (s)', fontsize=14)

    ax2.plot(error[2][:], error[0][:]* 52.5 - 1 + 0.05*52.5, color='blue', label='error')
    ax2.plot(disturbance[2][:], disturbance[0][:]* 52.5 , color='red', label ='impulse')
    ax2.plot(setPoint[2][:], setPoint[0][:], color='black', label='xd')
    #axs[0, 0].plot(controlSignal[2,:], controlSignal[0,:], color='blue')
    #ax2.tick_params(axis='both', which='major', labelsize=6)
    #ax2.get_xaxis().set_visible(False)
    ax2.set_ylim(-20, 20)
    ax2.set_yticks([-20, -10, 0, 10, 20])
    ax2.set_xticks([0, 5, 10, 15, 20])
    ax2.set_ylabel('error (mm)', fontsize=14)
    ax2.set_xlabel('Time (s)', fontsize=14)

    ax2.legend()
    ax2.set_xlim(0, 20)
    #ax2.

    ax1.grid()
    ax2.grid()

    plt.subplots_adjust(wspace=0, hspace=0)
    plt.tight_layout()
    

    save_folder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/ArraysACC' 
    file_name = 'XaxisPlot_PID.pdf' 
    dpi_value = 600  
    save_path = f"{save_folder}/{file_name}"
    plt.savefig(save_path, dpi=dpi_value)

def plotTimeVerification(): 
    import os 
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification')
    LQGTiming = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification/LQGTiming.npy')
    ServoTiming = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification/ServoTiming.npy')
    touchScreenReadingTiming = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification/touchScreenReadingTiming.npy')
    

    plt.plot(np.linspace(0, len(LQGTiming), len(LQGTiming)), LQGTiming, label='LQGTiming')
    plt.plot(np.linspace(0, len(ServoTiming), len(ServoTiming)), ServoTiming, label='ServoTiming')
    plt.plot(np.linspace(0, len(touchScreenReadingTiming), len(touchScreenReadingTiming)), touchScreenReadingTiming, label='touchScreenReadingTiming')
    plt.title('Time Verification')
    plt.xlabel('X-axis Label')
    plt.ylabel('Y-axis Label')
    plt.ylim([0, 0.03])
    plt.legend()
    plt.grid()

    save_folder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/timeVerification/' 

    file_name = 'timeVerification.png' 
    file_path = os.path.join(save_folder, file_name)

    plt.savefig(file_path)


    plt.show()

    print(f"Figure saved as {file_path}")

def plotTouchscreen(): 
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements')

def plot(): 
    os.chdir('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements')
    control = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/controlNengo.npy')
    error = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/errorNengo.npy')
    setpoint = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/setPointEnsembleNengo.npy')
    touchReadingNengo = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/touchScreenReadingNengo.npy')
    disturbance = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/disturbanceData.npy')
    touchReading = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/touchScreenReadingRaw.npy')
    frequencyTouchscreen = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/touchScreenReadingFrequency.npy')
    frequencyNengo = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/frequencyNengo.npy')
    frequencyServo = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/frequencyServos.npy')
    plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
    plt.rcParams['font.serif'] = 'DejaVu Serif'
    plt.rcParams['pdf.fonttype'] = 42
    # X - Axis
    fig, (ax1, ax2) = plt.subplots(2)
    # plot 1
    ax1.plot(control[2][:], control[0][:], color='red', label='ux')
    ax1.set_ylabel('ux (rad)', color = 'black', fontsize=14)
    ax1.set_yticks([-0.4, -0.2, 0, 0.2, 0.4])

    ax3 = ax1.twinx()
    ax3.plot(touchReadingNengo[2][:] , touchReadingNengo[0][:] * 52.5 - 1, color='blue', label='x')
    ax3.plot(setpoint[2][:], setpoint[0][:], color='black', label='xd')
    ax3.set_ylabel('x, xd (mm)', fontsize=14)
    ax3.set_yticks([-20, -10, 0, 10, 20])
    ax1.set_xticks([0, 5, 10, 15, 20])
    ax3.set_xticks([0, 5, 10, 15, 20])

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax3.get_legend_handles_labels()

    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax3.get_ylim()
    
    ax3.set_ylim(-20, 20)
    ax1.set_ylim(-0.50, 0.50)
    ax3.set_xlim(0, 20)
    ax1.set_xlim(0, 20)
    ax1.set_xlabel('Time (s)', fontsize=14)
    ax3.set_xlabel('Time (s)', fontsize=14)

    # plot 2
    ax2.plot(error[2][:], error[0][:]* 52.5 - 1 + 0.030*52.5, color='blue', label='error')
    ax2.plot(setpoint[2][:], setpoint[0][:], color='black', label='xd')
    ax2.plot(disturbance[2][:], disturbance[0][:]* 52.5  , color='red', label ='impulse')
    ax2.set_ylim(-20, 20)
    ax2.set_yticks([-20, -10, 0, 10, 20])
    ax2.set_xticks([0, 5, 10, 15, 20])
    ax2.set_ylabel('error (mm)', fontsize=14)
    ax2.set_xlabel('Time (s)', fontsize=14)
    ax2.legend()
    ax2.set_xlim(0, 20)
    
    ax1.grid()
    ax2.grid()
    plt.subplots_adjust(wspace=0, hspace=0)
    plt.tight_layout()

    savefolder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurementPlots'
    file_name = 'XaxisPlot.png'
    dpi_value = 300 
    save_path = f"{savefolder}/{file_name}"
    plt.savefig(save_path, dpi=dpi_value)

    # Y - Axis
    fig, (ax1, ax2) = plt.subplots(2)
    # plot 1
    ax1.plot(control[2][:], control[1][:], color='red', label='uy')
    ax1.set_ylabel('uy (rad)', color = 'black', fontsize=14)
    ax1.set_yticks([-0.4, -0.2, 0, 0.2, 0.4])

    ax3 = ax1.twinx()
    ax3.plot(touchReadingNengo[2][:] , touchReadingNengo[1][:] * 52.5 - 1, color='blue', label='y')
    ax3.plot(setpoint[2][:], setpoint[1][:], color='black', label='yd')
    ax3.set_ylabel('y, yd (mm)', fontsize=14)
    ax3.set_yticks([-20, -10, 0, 10, 20])
    ax1.set_xticks([0, 5, 10, 15, 20])
    ax3.set_xticks([0, 5, 10, 15, 20])

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax3.get_legend_handles_labels()

    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    y1_min, y1_max = ax1.get_ylim()
    y2_min, y2_max = ax3.get_ylim()
    
    ax3.set_ylim(-20, 20)
    ax1.set_ylim(-0.50, 0.50)
    ax3.set_xlim(0, 20)
    ax1.set_xlim(0, 20)
    ax1.set_xlabel('Time (s)', fontsize=14)
    ax3.set_xlabel('Time (s)', fontsize=14)

    # plot 2
    ax2.plot(error[2][:], error[1][:]* 52.5 - 1 + 0.015*52.5, color='blue', label='error')
    ax2.plot(setpoint[2][:], setpoint[1][:], color='black', label='yd')
    ax2.plot(disturbance[2][:], disturbance[1][:]* 52.5  , color='red', label ='impulse')
    ax2.set_ylim(-20, 20)
    ax2.set_yticks([-20, -10, 0, 10, 20])
    ax2.set_xticks([0, 5, 10, 15, 20])
    ax2.set_ylabel('error (mm)', fontsize=14)
    ax2.set_xlabel('Time (s)', fontsize=14)
    ax2.legend()
    ax2.set_xlim(0, 20)
    
    ax1.grid()
    ax2.grid()
    plt.subplots_adjust(wspace=0, hspace=0)
    plt.tight_layout()

    savefolder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurementPlots'
    file_name = 'YaxisPlot.png'
    dpi_value = 300 
    save_path = f"{savefolder}/{file_name}"
    plt.savefig(save_path, dpi=dpi_value)

    fig, (ax1, ax2) = plt.subplots(2)
    # Plot of the frequency of the touchscreen, nengo and servo
    ax1.plot(frequencyNengo[2][:], frequencyNengo[0][:], color='blue', label='Nengo')
    ax1.plot(frequencyServo[2][:], frequencyServo[0][:], color='green', label='Servo')
    ax1.plot(frequencyTouchscreen[2][:], frequencyTouchscreen[0][:], color='red', label='Touchscreen')

    min = np.min([len(frequencyServo[0][:]), len(frequencyNengo[0][:]), len(frequencyTouchscreen[0][:])])
    totalFreq = 1/frequencyTouchscreen[0][:min] + 1/frequencyNengo[0][:min] + 1/frequencyServo[0][:min] 
    ax2.plot(frequencyNengo[2][:min], 1/totalFreq, color='blue', label='Total Frequency')
    ax2.set_xlim(0, 20)
    ax1.set_xlim(0, 20)
    ax1.set_ylim(0, 400)
    ax2.legend()
    ax1.legend()

    
    plt.subplots_adjust(wspace=0, hspace=0)
    plt.tight_layout()

    savefolder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurementPlots'
    file_name = 'frequency.png'
    dpi_value = 300 
    save_path = f"{savefolder}/{file_name}"
    plt.savefig(save_path, dpi=dpi_value)

    
    

if __name__ == '__main__':
    plot()