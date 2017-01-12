# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 17:41:10 2017

@author: cunnia3
"""

class PID:
    """
    discrete PID control for finding next command u[k]
    u[k] = au[k-1] + be[k] + ce[k-1] + de[k-2]
    """
    def __init__(self, a=0.0, b=0.0, c=0.0, d=0.0):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
         
        self.set_point = 0     # reference
        self.e = 0             # current error
        self.pe = 0            # previous error
        self.ppe = 0           # error at e[k-2]
        self.u = 0             # current command
        self.pu = 0            # previous command
 
    def udpate(self, measurement):
        """ Use measurement to come up with new command """
        self.pu = self.u
        self.ppe = self.pe
        self.pe = self.e
        
        self.e = self.set_point - measurement         
        self.u = self.a * self.pu + self.b*self.e + self.c*self.pe + self.d*self.ppe
        
        return self.u
        
    def setPoint(self, set_point):
        self.set_point = set_point
        
    def getPoint(self):
        return self.set_point
        
    def getError(self):
        return self.e