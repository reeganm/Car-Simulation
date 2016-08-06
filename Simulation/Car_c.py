# -*- coding: utf-8 -*-

import numpy as np

class Car_c:
    """ Class containing model of Car """
    
    #Constants
    Mass = 0
    WheelDiameter = 0
    
    GearRatio = 0
    GearEfficiency = 0
    
    RollingResistanceCoefficient = 0
    ConstantDragForce = 0

    AreodynamicDragCoefficient = 0
    
    FrontalArea = 0
    
    
    #Variable arrays
    CarAcceleration = ''
    CarSpeed = ''
    CarDrag = ''
 
    
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
