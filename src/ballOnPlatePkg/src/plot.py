import os
import numpy as np
import matplotlib.pyplot as plt

# Define the file name and path
def plotSimpleEnsemble():
    filename = 'XY_E'
    current_dir = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src'
    folder_path = os.path.join(current_dir, 'measurements')
    array_path = os.path.join(folder_path, f'{filename}.npy')
    fig_path = os.path.join(folder_path, f'{filename}.png')

    # Load the data from the file
    data = np.load(array_path)

    # Extract the data from the loaded array
    data1 = data[0,:]
    data2 = data[1,:]
    timestamp = data[2,:]

    # Plot the data
    plt.plot(np.linspace(0, len(data1), len(data1)), data1, label='Data 1')
    plt.plot(np.linspace(0, len(data2), len(data2)), data2, label='Data 2')
    plt.xlabel('Time')
    plt.ylabel('Data')
    plt.legend()

    plt.savefig(fig_path, dpi=300)

if __name__ == '__main__':
    plotSimpleEnsemble()