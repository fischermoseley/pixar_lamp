import numpy as np
import matplotlib.pyplot as plt

trajectory = np.load('trajectory.npy')

print(trajectory)
print(trajectory.shape)

plt.plot(trajectory[:,0], label = 'th1')
plt.plot(trajectory[:,1], label = 'dth1')
plt.plot(trajectory[:,2], label = 'th2')
plt.plot(trajectory[:,3], label = 'dth2')
plt.legend()
plt.show()