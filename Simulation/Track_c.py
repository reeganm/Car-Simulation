# -*- coding: utf-8 -*-

import numpy as np
import math
from numba import jit
import matplotlib.pyplot as plt

class Track_c:
    """ Class containing model of Track Conditions """
    
    #Constants
    AirPressure = 0
    RelativeHumidity = 0
    Temperature = 0
    
    AirDensity = 0
    
    Incline = [];
    DataPoints = ''
    SimulationTime = []
    TimeInterval = []
    TrackLength = [];
    
    #Variable arrays
    
        
    def __init__(self,SimulationTime,TimeInterval,TrackLength):
        #class constructor
        print('Track Object Created')
        
        self.DataPoints = math.floor(SimulationTime/TimeInterval) 
        self.SimulationTime = SimulationTime
        self.TimeInterval = TimeInterval
        self.TrackLength = TrackLength
        #Allocate RAM
        self.Incline = np.zeros(TrackLength)

    @jit
    def calc_AirDensity(self):
        self.AirDensity=((self.AirPressure-(self.RelativeHumidity/100)*math.exp(-42800/8.314462*(1/(self.Temperature+273.15)-1/373.15)+math.log(101.325)))*28.9644+(self.RelativeHumidity/100)*math.exp(-42800/8.314462*(1/(self.Temperature+273.15)-1/373.15)+math.log(101.325))*18.02)/(8.314462*(self.Temperature+273.15))
        return(self.AirDensity)
    
    @jit        
    def smoothtrack(self,k):
        for n in range(k,len(self.Incline)-k):
            self.Incline[n] = np.sum(self.Incline[(n-k):(n+k)]) / k / 2
    
    @jit
    def calc_Incline(self,distance):
        distance = int(distance)
        if distance >= len(self.Incline):
            distance = len(self.Incline)-1
        incline = self.Incline[distance]
        return(incline)
    
    def plot_Profile(self):
        plt.plot(self.Incline)
        plt.ylabel('Slope (deg)')
        plt.xlabel('Distance (m)')
        plt.title('Track Profile')
        plt.show()