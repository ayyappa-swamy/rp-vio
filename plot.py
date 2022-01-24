import numpy as np 
import matplotlib.pyplot as plt


x = [ 8000000 , 
1000000,
125000
  ]



y1 = [ 
45,
4.5,
0.45
]

y2 = [
1836,
231,
28
]


y1 = np.asarray( y1)
y2 = np.asarray( y2)
x = np.asarray(x)


plt.subplot(1, 2, 1)
plt.plot(x, y1 , label='Analytical Collision Check')
plt.ticklabel_format(useOffset=True)
plt.xlabel("Number of Points ")
plt.ylabel("Time (ms)")
plt.legend()

plt.subplot(1, 2, 2)

plt.plot(x, y2 , label = 'EDT')
plt.ticklabel_format(useOffset=True)

plt.xlabel("Number of Points")
plt.ylabel("Time (ms)")
plt.legend()
plt.show()