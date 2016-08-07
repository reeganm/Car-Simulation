# -*- coding: utf-8 -*-

import numpy as np
import math

class Track_c:
    """ Class containing model of Track Conditions """
    
    #Constants
    AirPressure = 0
    RelativeHumidity = 0
    Temperature = 0
    
    AirDensity = 0
    
    Incline = 0
    DataPoints = ''
    
    
    #Variable arrays
    
        
    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('Track Object Created')
        
        self.DataPoints = math.floor(SimulationTime/TimeInterval) 
        self.SimulationTime = SimulationTime
        self.TimeInterval = TimeInterval
        
        #Allocate RAM

        
    def calc_AirDensity(self):
        self.AirDensity=((self.AirPressure-(self.RelativeHumidity/100)*math.exp(-42800/8.314462*(1/(self.Temperature+273.15)-1/373.15)+math.log(101.325)))*28.9644+(self.RelativeHumidity/100)*math.exp(-42800/8.314462*(1/(self.Temperature+273.15)-1/373.15)+math.log(101.325))*18.02)/(8.314462*(self.Temperature+273.15))
        return(self.AirDensity)