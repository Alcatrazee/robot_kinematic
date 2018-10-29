# -*- coding: utf-8 -*-
#Author:Ma Jian
#Date:2018.10.29
#Version:1.0
#Comment:This is the script that make things done,which will calculate angles 
#        needed to reach the given posture
from functions import *
import numpy as np
from scipy import linalg

#just to make data more easy understand
np.set_printoptions(suppress=True)                 

#function: inverse_kinematic
#input: a homogenous represantation posture of a robot end_effector
#output: the angle vector needed to reach the posture 
def inverse_k(g_st_theta):
    # error tolarance
    E = 0.000002     
    # maximun times of iterate                        
    limit_of_iterate = 20
    # initial posture of tool frame
    g_st0 = np.matrix([ [1 ,0 ,0 ,533],
                        [0  ,0 ,1 ,0],
                        [0 ,-1 ,0 ,889.1],
                        [0  ,0 ,0 ,1]])
    # rotation axis of each joint(by row)
    omega = np.matrix([ [0,0,0,1,0,1],
                        [0,1,1,0,1,0],
                        [1,0,0,0,0,0]])
    # matrix formed by points in line of ratation axis
    q = np.matrix([ [0,0,0,0,451,533],
                    [0,0,0,0,0,0],
                    [0,399.1,847.1,889.1,889.1,889.1]])
    # initial state vector(just 6 angles)
    theta_vector = np.zeros((1,6))
    # prepare parameters for PoE calculation
    v = cross(-omega[:,0],q[:,0])
    for x in range(1,6):
        v = np.hstack((v,cross(-omega[:,x],q[:,x])))
    # twist coordinate
    epsi_vector = make_epsilo(v,omega)
    # prepare parameters for PoE calculation
    P = np.zeros((3,6))
    
    # apply Newton's method
    for i in range(30):
        # forward kinematic things
        R1 = make_rotation(omega[:,0],theta_vector[0,0])
        R2 = make_rotation(omega[:,1],theta_vector[0,1])
        R3 = make_rotation(omega[:,2],theta_vector[0,2])
        R4 = make_rotation(omega[:,3],theta_vector[0,3])
        R5 = make_rotation(omega[:,4],theta_vector[0,4])
        R6 = make_rotation(omega[:,5],theta_vector[0,5])

        P = make_P(omega[:,0],v[:,0],theta_vector[0,0])
        for x in range(1,6):
            P = np.hstack((P,make_P(omega[:,x],v[:,x],theta_vector[0,x]))) 
        
        exp1 = make_PoE(R1,P[:,0])
        exp2 = make_PoE(R2,P[:,1])
        exp3 = make_PoE(R3,P[:,2])
        exp4 = make_PoE(R4,P[:,3])
        exp5 = make_PoE(R5,P[:,4])
        exp6 = make_PoE(R6,P[:,5])
        
        # get epsilo_theta(k)
        epsi_th_k = vee(linalg.logm(exp1*exp2*exp3*exp4*exp5*exp6*g_st0*linalg.inv(g_st_theta)))
        # get Jacobian matrix
        Jacobian = get_Jacobian(epsi_vector,exp1,exp2,exp3,exp4,exp5)
        # get persuado Jacobian matrix
        ps_Jacobian = linalg.pinv(Jacobian)
        # iterate theta vector ot vector(k+1)
        theta_vector = theta_vector - np.transpose(ps_Jacobian*epsi_th_k)
        # calculate the norm of epsilo_theta(k)
        Norm_of_phi = linalg.norm(epsi_th_k)
        # check if we have reached the posture
        if Norm_of_phi<E:
            succeeded=1
            break
            
        
    print('norm of phi:'+str(Norm_of_phi))
    if succeeded==1:
        print('inverse result:'+str(theta_vector))
        return theta_vector
    else:
        print('cannot solve the equation,maybe the posture is out of range.')
        return np.matrix([[0,0,0,0,0,0]])
    