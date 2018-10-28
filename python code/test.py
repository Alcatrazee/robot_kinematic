# -*- coding: utf-8 -*-
"""
Created on Mon Oct 29 00:21:32 2018

@author: marki
"""
from functions import *
import numpy as np
from scipy import linalg

a = np.matrix(np.matrix([[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]]))
print(Get_Adg(a))