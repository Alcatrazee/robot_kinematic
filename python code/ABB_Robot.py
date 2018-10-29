# -*- coding: utf-8 -*-
from forward_kinematic import *
from inverse_kinematic import *
import numpy as np

#Before running,you need to make sure you have installed numpy,scipy,matplotlib.
#All you need to do is to alternate the angle of each joint on below 
#Set the theta vector to determine the angle of each joint
theta_vector = np.matrix([[np.radians(90),np.radians(50),np.radians(00),np.radians(30),np.radians(40),np.radians(0)]])
#forward result is the posture in SE(3)
forward_result = forward_kinematic(theta_vector)
print('forward result')
print(forward_result)
print('\n')
#use the result of forward kinematic as input to calculate the inverse kinematic
theta_vector_by_inverse = inverse_k(forward_result)
#result is the theta_vector as we input in line6
print('theta vector given:'+str(theta_vector))
forward_kinematic(theta_vector_by_inverse)