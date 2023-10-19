import matplotlib.pyplot as plt
import numpy as np

# read weights from file
weights = np.load('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/measurements/weightsNengo.npy')

# plot weights
plt.plot(weights[:, 0])
plt.xlabel('Neuron Index')
plt.ylabel('Weight Value')
plt.title('Weight Values from Neurons')
plt.savefig('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/plots/weightsNengo.png')

