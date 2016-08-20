# -*- coding: utf-8 -*-

import numpy as np
import math
from numba import jit

class SuperCapacitor_c:
    """ Class containing model of Super Capacitor """
    
    #Constants
    Capacitance = 0;
	
	SimulationTime = 0;
	TimeInterval = 0;
	DataPoints = 0;
	
    #Variable arrays
	TimeEllapsed = [];
    Charge = [];
	Voltage =[];
	
    #equations
    
        
    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('Super Capacitor Object Created')
        
		self.SimulationTime = SimulationTime
        self.Timeinterval = TimeInterval
        self.DataPoints = math.floor(SimulationTime/TimeInterval)         
        
        #make time array for plotting
        self.TimeEllapsed = np.arange(self.DataPoints)*TimeInterval
		
		self.Charge = np.zeros(self.DataPoints)
		self.Voltage = np.zeros(self.DataPoints)
	
	@jit
	def DrainCaps(self,CurrentCharge,Current,TimeInterval):
		Charge = CurrentCharge - Current*TimeInterval
		return(Charge)
	
	@jit
	def CalcVoltage(self,Charge):
		Voltage = Charge / self.Capacitance
		return(Voltage)