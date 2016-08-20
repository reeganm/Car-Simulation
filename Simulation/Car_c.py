# -*- coding: utf-8 -*-

import numpy as np
import math
from numba import jit
import matplotlib.pyplot as plt

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
        self.DataPoints = math.floor(SimulationTime/TimeInterval)         
        
        #make time array for plotting
        self.TimeEllapsed = np.arange(self.DataPoints)*TimeInterval
        
        #Allocate RAM
        self.Acceleration = np.zeros(self.DataPoints)
        self.Speed = np.zeros(self.DataPoints)
        self.DistanceTravelled = np.zeros( self.DataPoints )

    @jit
    def calc_AirDrag(self,AirDensity,Speed):
        AirDrag = 0.5*self.AreodynamicDragCoefficient*self.FrontalArea*AirDensity*math.pow(Speed,2)
		return(AirDrag)
        
		
    ## Plotting ##
    def plot_DistanceTime(self):
        plt.plot(self.TimeEllapsed, self.DistanceTravelled)
        plt.xlabel('Time')
        plt.ylabel('Distance (m)')
        plt.title('Car')
        plt.show()
        
    def plot_SpeedTime(self):
        plt.plot(self.TimeEllapsed, self.Speed*3.6)
        plt.xlabel('Time')
        plt.ylabel('Speed (km.h)')
        plt.title('Car')
        plt.show()
        
    def plot_AccelerationTime(self):
        plt.plot(self.TimeEllapsed, self.Acceleration)
        plt.xlabel('Time')
        plt.ylabel('Acceleration (m/s2)')
        plt.title('Car')
        plt.show()