# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 13:26:12 2017

@author: cats
"""

import numpy as np

my_data = genfromtxt('ar_data.csv', delimiter=',')
cov = np.cov(np.transpose(my_data))
print cov