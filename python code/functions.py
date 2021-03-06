# -*- coding: utf-8 -*-
#Author:Ma Jian
#Date:2018.10.29
#Version:1.0
#Comment: this script contains all functions needed for the inverse and forward
#         kinematic
#  
import numpy as np
import matplotlib.pyplot as plt 
from scipy import linalg 

# get rotation matrix around currtant axis
def make_rotation(axis,theta):
    R = np.eye(3)+hat_3D(axis)*np.sin(theta)+(hat_3D(axis)*hat_3D(axis))*(1-np.cos(theta))
    return R

# calculat hat of a vector
def hat_3D(column_vector):
    out = np.matrix([[0,-column_vector[2,0],column_vector[1,0]],
                     [column_vector[2,0],0,-column_vector[0,0]],
                     [-column_vector[1,0],column_vector[0,0],0]])
    return out

# define a cross product of my own,np.cross is different to matlab's cross product
def cross(a,b):
    a_hat = hat_3D(a)
    out = a_hat*b
    return out

# make P for the PoE
# formular P = (I-R)*(w x v) + (w*w'*v*theta)
def make_P(omega,v,theta):
    if np.ndim(omega)!=0:
        out = (np.eye(3)-make_rotation(omega,theta))*cross(omega,v)+(omega*np.transpose(omega)*v*theta)
    else:
        out = v*theta
    return out

# make PoE coordinate with R and P
def make_PoE(R,P):
    bottom_vector = np.matrix([[0,0,0,1]])
    upper_matrix = np.hstack((R,P))
    out = np.vstack((upper_matrix,bottom_vector))
    return out

# make a plot to display 
def draw_figure(point_matrix):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    x = [point_matrix[0,0],point_matrix[0,1],point_matrix[0,2],point_matrix[0,3],point_matrix[0,4],point_matrix[0,5]]
    y = [point_matrix[0,0],point_matrix[1,1],point_matrix[1,2],point_matrix[1,3],point_matrix[1,4],point_matrix[1,5]]
    z = [point_matrix[0,0],point_matrix[2,1],point_matrix[2,2],point_matrix[2,3],point_matrix[2,4],point_matrix[2,5]]
    ax.plot(x,y,z)
    ax.scatter(x,y,z,s=60,c='r',marker='o')
    plt.axis([-1000,1000,-1000,1000])
    plt.show()

# epsilo coordinate
def make_epsilo(v,omega):
    epsi = np.vstack((v,omega))
    return epsi

# get adjoint transformation matrix
def Get_Adg(exp):
    left_up = exp[0:3,0:3]
    right_up = hat_3D(exp[0:3,3])*exp[0:3,0:3]
    left_bottom = np.zeros((3,3))
    right_bottom = exp[0:3,0:3]
    out = np.vstack((np.hstack((left_up,right_up)),np.hstack((left_bottom,right_bottom))))
    return out
# operation vee,or remove hat
def vee(se3):
    omega = np.matrix([[se3[2,1]],[se3[0,2]],[se3[1,0]]])
    v = np.matrix([[se3[0,3]],[se3[1,3]],[se3[2,3]]])
    out = np.vstack((v,omega))
    return out

# get Jacobian matrix
def get_Jacobian(epsi_all,exp1,exp2,exp3,exp4,exp5):
    out = epsi_all[:,0]
    out = np.hstack((out,Get_Adg(exp1)*epsi_all[:,1]))
    out = np.hstack((out,Get_Adg(exp1*exp2)*epsi_all[:,2]))
    out = np.hstack((out,Get_Adg(exp1*exp2*exp3)*epsi_all[:,3]))
    out = np.hstack((out,Get_Adg(exp1*exp2*exp3*exp4)*epsi_all[:,4])) 
    out = np.hstack((out,Get_Adg(exp1*exp2*exp3*exp4*exp5)*epsi_all[:,5]))
    return out