import matplotlib.pyplot as plt
import numpy as np
x = [ 1,2,3,4,5,6,7 ,8, 9]
y = [ 2.87 ,0.271, 0.1088 ,0.075 , 0.082 , 0.073 ,  0.069  ,  0.055  ,  0.057   ]
y = np.asarray(y, dtype='float')
val = np.sum(y)
y /= 2.87

print(y)
plt.ylabel("Normalized Cost " , fontname="Times New Roman",  fontsize=16)
plt.xlabel(" Iterations " , fontname="Times New Roman",  fontsize=16)
plt.title("CEM Convergance Analysis" , fontname="Times New Roman",fontweight="bold" , fontsize=16)
plt.plot( x, y)
plt.show()
