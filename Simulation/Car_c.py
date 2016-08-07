# -*- coding: utf-8 -*-

import numpy as np
import math

class Car_c:
    """ Class containing model of Car """
    
    #Constants
    Mass = 0  #kg
    
    WheelDiameter = 0  #m
    WhellInertia = 0 #kgm^2
    
    GearRatio = 0
    GearEfficiency = 0 #fraction
    GearInertia = 0
    
    RollingResistanceCoefficient = 0
    BearingResistance = 0 #N

    AreodynamicDragCoefficient = 0
    AirDrag = 0
    
    FrontalArea = 0 #m^2
    
    DataPoints = ''
    SimulationTime = ''
    TimeInterval = ''
    
    #Variable arrays
    Acceleration = ''
    Speed = ''
    DistanceTravelled = ''
    
    #equations
    
        
    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('Car Object Created')
        
        self.SimulationTime = SimulationTime
        self.Timeinterval = TimeInterval
        
        #Allocate RAM
        self.DataPoints = math.floor(SimulationTime/TimeInterval)       
        self.Acceleration = np.zeros(self.DataPoints)
        self.Speed = np.zeros(self.DataPoints)
        self.DistanceTravelled = np.zeros( self.DataPoints )


    def calc_AirDrag(self,AirDensity,Index):
        self.AirDrag = 0.5*self.AreodynamicDragCoefficient*self.FrontalArea*AirDensity*math.pow(self.Speed[Index],2)