from forward_kinematic import *
import numpy as np
from inverse_kinematic import *

#theta_vector = np.matrix([[np.radians(00),np.radians(00),np.radians(00),np.radians(0),np.radians(0),np.radians(0)]])
#forward_result = forward_kinematic(theta_vector)
#print(forward_result)
#g_st0 = np.matrix([[1 ,0 ,0 ,533],
#                    [0  ,0 ,1 ,0],
#                    [0 ,-1 ,0 ,889.1],
#                    [0  ,0 ,0 ,1]])
g_st_theta =     np.matrix([[0.1736 ,  -0.9848    ,     0 , 357.9164],
         [0    ,     0   , 1.0000    ,     0],
   [-0.9848 ,  -0.1736   ,      0  ,269.4701],
        [ 0  ,       0     ,    0  ,  1.0000]])
inverse_k(g_st_theta)