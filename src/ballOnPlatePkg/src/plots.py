#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, signal
import matplotlib.pyplot as plt 

def plotTwoAxis(data1, data2, timestamp, title, xLabel, yLabel, figure_name, limit=True): 
    plt.figure()
    plt.plot(timestamp, data1, label='Data 1')
    plt.plot(timestamp, data2, label='Data 2')
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
    plt.savefig(figure_path, dpi=300)
    plt.close()
