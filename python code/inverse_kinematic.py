from functions import *
import numpy as np
from scipy import linalg

np.set_printoptions(suppress=True)
def inverse_k(g_st_theta):
    E = 0.000002
    limit_of_iterate = 20

    g_st0 = np.matrix([ [1 ,0 ,0 ,533],
                        [0  ,0 ,1 ,0],
                        [0 ,-1 ,0 ,889.1],
                        [0  ,0 ,0 ,1]])

    omega = np.matrix([ [0,0,0,1,0,1],
                        [0,1,1,0,1,0],
                        [1,0,0,0,0,0]])
    q = np.matrix([ [0,0,0,0,451,533],
                    [0,0,0,0,0,0],
                    [0,399.1,847.1,889.1,889.1,889.1]])
    
    theta_vector = np.zeros((1,6))

    v = cross(-omega[:,0],q[:,0])
    for x in range(1,6):
        v = np.hstack((v,cross(-omega[:,x],q[:,x])))
    epsi_vector = make_epsilo(v,omega)
    P = np.zeros((3,6))
    
    for i in range(30):
        
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
        
        epsi_th_k = vee(linalg.logm(exp1*exp2*exp3*exp4*exp5*exp6*g_st0*linalg.inv(g_st_theta)))
        
        Jacobian = get_Jacobian(epsi_vector,exp1,exp2,exp3,exp4,exp5)

        ps_Jacobian = linalg.pinv(Jacobian)

        theta_vector = theta_vector - np.transpose(ps_Jacobian*epsi_th_k)
        Norm_of_phi = linalg.norm(epsi_th_k)

        if Norm_of_phi<E:
            break

    print(Norm_of_phi)
    print(theta_vector)
