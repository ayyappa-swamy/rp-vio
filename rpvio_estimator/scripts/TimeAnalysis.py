import numpy as np
import matplotlib.pyplot as plt 


QTime_125000_ours = np.asarray([ 0.0240,
0.00163 , 
0.00223,
0.00155,
0.0011])

QTime_8000_ours = np.asarray( [ 0.00120,
0.000688,
0.0022,
0.0023,
0.02195])

QTime_8000_EDT = np.asarray( [ 0.001,
0.001,
0.001,
0.001,
0.001])


QTime_125000_EDT = np.asarray([ 0.019,
0.019,
0.019,
0.019,
0.019])

NumPlanes = np.asarray([ 1000,
500 ,
100,
70,
50])


plt.plot( NumPlanes , QTime_125000_ours*1000 ,  label='Geometric Representation' , marker='*' , linestyle='--' )
# plt.plot(NumPlanes ,  QTime_8000_ours ,  label= '(Proposed) Query time for 8000 points ', marker='*', linestyle='--' )
# plt.plot( NumPlanes , QTime_8000_EDT ,  label= '(EDT) Query time for 8000 points ' , marker='o')
plt.plot(NumPlanes , QTime_125000_EDT*1000 ,  label='Euclidian Distance Transform(EDT)' , marker ='o'  )
plt.xlabel('Number of Planes/ Number of voxels in a map ')
plt.ylabel('Query Time (ms)')
plt.yticks(np.arange(0, 30, 3)) 
plt.legend(fontsize=8)
plt.title("Query Time Analysis")
plt.show()
