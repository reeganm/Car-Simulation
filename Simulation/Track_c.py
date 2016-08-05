# -*- coding: utf-8 -*-

import numpy as np

class Motor_c:
    """ Class containing model of Track Conditions """
    
    #Constants
    AirPressure = 0
    Humidity = 0
    Temperature = 0
    
    

    
    #Variable arrays
    TrackIncline = ''
    
    #equations
    
        
    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('Car Object Created')
        
        #Allocate RAM
        DataPoints = SimulationTime/TimeInterval
        DataPoints = int(DataPoints)        
        self.CarAcceleration = np.zeros((DataPoints,1))
        self.CarSpeed = np.zeros((DataPoints,1))
        self.CarDrag = np.zeros((DataPoints,1))
