import numpy as np
from functions import *
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

def forward_kinematic(theta_vector):
    np.set_printoptions(suppress=True)
    #parameters
    g_st0 = np.matrix([[1 ,0 ,0 ,533],
                    [0  ,0 ,1 ,0],
                    [0 ,-1 ,0 ,889.1],
                    [0  ,0 ,0 ,1]])

    omega = np.matrix([[0,0,0,1,0,1],
                    [0,1,1,0,1,0],
                    [1,0,0,0,0,0]])
    q = np.matrix([[0,0,0,0,451,533],
                [0,0,0,0,0,0],
                [0,399.1,847.1,889.1,889.1,889.1]])

    #theta_vector = np.matrix([[np.radians(00),np.radians(-90),np.radians(-90),np.radians(0),np.radians(0),np.radians(0)]])

    # forward kinematic
    R1 = make_rotation(omega[:,0],theta_vector[0,0])
    R2 = make_rotation(omega[:,1],theta_vector[0,1])
    R3 = make_rotation(omega[:,2],theta_vector[0,2])
    R4 = make_rotation(omega[:,3],theta_vector[0,3])
    R5 = make_rotation(omega[:,4],theta_vector[0,4])
    R6 = make_rotation(omega[:,5],theta_vector[0,5])

    #v = np.matrix([np.cross(-omega[:,0],q[:,0]),np.cross(-omega[:,1],q[:,1]),np.cross(-omega[:,2],q[:,2]),np.cross(-omega[:,3],q[:,3]),np.cross(-omega[:,4],q[:,4]),np.cross(-omega[:,5],q[:,5]])
    v = cross(-omega[:,0],q[:,0])
    for x in range(1,6):
        v = np.hstack((v,cross(-omega[:,x],q[:,x])))

    P = make_P(omega[:,0],v[:,0],theta_vector[0,0])
    for x in range(1,6):
        P = np.hstack((P,make_P(omega[:,x],v[:,x],theta_vector[0,x])))

    exp1 = make_PoE(R1,P[:,0])
    exp2 = make_PoE(R2,P[:,1])
    exp3 = make_PoE(R3,P[:,2])
    exp4 = make_PoE(R4,P[:,3])
    exp5 = make_PoE(R5,P[:,4])
    exp6 = make_PoE(R6,P[:,5])

    g_st_theta = exp1*exp2*exp3*exp4*exp5*exp6*g_st0

    """ print(exp1)
    print(exp2)
    print(exp3)
    print(exp4)
    print(exp5)
    print(exp6)
    print(g_st_theta) """

    point_matrix = np.transpose(np.matrix([[0,0,0,1]]))
    point_matrix = np.hstack((point_matrix,exp1*np.transpose(np.matrix([[0,0,399.1,1]]))))
    point_matrix = np.hstack((point_matrix,exp1*exp2*exp3*np.transpose(np.matrix([[0,0,847.1,1]]))))
    point_matrix = np.hstack((point_matrix,exp1*exp2*exp3*np.transpose(np.matrix([[0,0,889.1,1]]))))
    point_matrix = np.hstack((point_matrix,exp1*exp2*exp3*exp4*exp5*np.transpose(np.matrix([[451,0,889.1,1]]))))
    point_matrix = np.hstack((point_matrix,g_st_theta[:,3]))

    #print(point_matrix)
    draw_figure(point_matrix)
    return g_st_theta




