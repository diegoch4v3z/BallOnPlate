
import os
import numpy as np

def saveActivity(data1, data2, timestamp, filename): 
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'measurements')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    array_path = os.path.join(folder_path, f'{filename}.npy')
    arrayCSV_path = os.path.join(folder_path, f'{filename}.csv')

    stacked_array = np.vstack((data1, data2, timestamp))
    np.save(array_path, stacked_array)
    np.savetxt(arrayCSV_path, stacked_array, delimiter=',' )