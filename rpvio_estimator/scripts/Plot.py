import matplotlib.pyplot as plt
import numpy as np
x = [ 1,2,3,4,5,6,7 ,8, 9]

y1 = [ 2.87 ,1.271, 0.81088 ,0.075 , 0.082 , 0.073 ,  0.069  ,  0.055  ,  0.057   ]


y2 = [ 21 , 4 , 1 , 0.5  , 0.1 , 0.1 , 0.01 , 0.01 , 0.01 ]


y3 = np.array([ 1651 , 513 ,  300 , 32 , 0.460499 , 0.564 , 0.5941 , 0.580 , 0.5862 ])

y1 = np.asarray(y1, dtype='float')
y2 = np.asarray(y2, dtype='float')





# val = np.sum(y)
y1 /= np.amax(y1)
y2 /= np.amax(y2)
y3 /= np.amax(y3)

plt.ylabel("Normalized Cost/Trace of Covariance " , fontname="Times New Roman",  fontsize=16)
plt.xlabel(" Iterations " , fontname="Times New Roman",  fontsize=16)
plt.title("CEM Convergance Analysis" , fontname="Times New Roman",fontweight="bold" , fontsize=16)
plt.plot( x, y1 , label='Normalized Cost STOMP perturbation' , linestyle='--')
plt.plot( x, y2 , label='Trace of Covariance (Polynomial perturbation)' , linestyle='dashdot') 
plt.plot( x, y3 , label='Normalized Cost (Polynomial perturbation)' , linestyle='-') 

plt.legend()

plt.show()
