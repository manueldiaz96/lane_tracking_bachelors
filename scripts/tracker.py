#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
from numpy import dot
from scipy.linalg import inv, block_diag



class Tracker(): # seguidor basado en Kalman Filter
    def __init__(self):


        # Inicializar parametros del tracker
        self.id = 0
        self.box = []
        self.hits = 0
        self.no_losses = 0
        self.x_state=[]
        self.dt = 1.   # time interval

        # asumiedo modelo de velocidad constante
        self.F= np.array([[1, self.dt],
                          [0,    1   ]])

        # Matrix de medición
        self.H = np.array([[1, 0]])


        # inicializar covarianza del estado
        self.L = 100.0
        self.P = np.diag(self.L*np.ones(2))


        # inicializar covarianza del proceso
        self.Q_comp_mat = np.array([[self.dt**4/2., self.dt**3/2.],
                                    [self.dt**3/2., self.dt**2]])
        # self.Q = block_diag(self.Q_comp_mat, self.Q_comp_mat,
        #                     self.Q_comp_mat, self.Q_comp_mat)

        self.Q = self.Q_comp_mat

        # Inicializar covarianza de la medición
        self.R_ratio = 1.0/16.0
        self.R_diag_array = self.R_ratio * np.array([self.L])
        self.R = np.diag(self.R_diag_array)


    def update_R(self):
        R_diag_array = self.R_ratio * np.array([self.L])
        self.R = np.diag(R_diag_array)


    def kalman_filter(self, z):
        '''
        Filtro
        '''
        #z = [[]]

        x = self.x_state
        # Predict
        x = dot(self.F, x)
        self.P = dot(self.F, self.P).dot(self.F.T) + self.Q


        #Update
        S = dot(self.H, self.P).dot(self.H.T) + self.R
        print(S.shape)
        K = dot(self.P, self.H.T).dot(inv(S)) # Kalman gain
        y = z - dot(self.H, x) # residual
        x += dot(K, y)
        self.P = self.P - dot(K, self.H).dot(self.P)
        self.x_state = x.astype(int) # convert to integer coordinates
                                     #(pixel values)

    def predict_only(self):
        '''
        Solo predicción
        '''
        x = self.x_state
        # Predict
        x = dot(self.F, x)
        self.P = dot(self.F, self.P).dot(self.F.T) + self.Q
        self.x_state = x.astype(int)



if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import glob
    
    #print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")

    # Creat an instance
    trk = Tracker()
    # Test R_ratio
    trk.R_ratio = 1.0/16

    trk.update_R()
    xx = 0

    trk.x_state = np.array([[0, 0]]).T
    trk.predict_only()
    #print(trk.x_state)

    # Update measurement noise covariance matrix
    for x in xrange(100,200):
    	x_st = [x]
    	z = np.expand_dims(x_st, axis=0).T
    	print "z:", z
    	trk.kalman_filter(z)
    	xx = trk.x_state;
    	print(xx[0,0])
    # Initial state
    
