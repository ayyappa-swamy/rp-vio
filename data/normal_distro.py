import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("./.ros/estim_error.txt")
errors = np.degrees(np.arccos(1 - data))

plt.plot(errors)
plt.xlabel("number of view points")
plt.ylabel("absolute angle error (degrees) of normals")
plt.show()

y, x = np.histogram(errors, bins=np.arange(45), density=True)
plt.plot(x[:-1], y)
plt.xlabel("absolute angle error (degrees) of normals")
plt.ylabel("probability")
plt.show()
