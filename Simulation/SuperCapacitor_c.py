# -*- coding: utf-8 -*-

import numpy as np
import math
from numba import jit
import matplotlib.pyplot as plt

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
    Current = [];
    PowerOut = [];
    
    #equations
    
        
    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('Super Capacitor Object Created')
        
        self.SimulationTime = SimulationTime
        self.TimeInterval = TimeInterval
        self.DataPoints = math.floor(SimulationTime/TimeInterval)         
        
        #make time array for plotting
        self.TimeEllapsed = np.arange(self.DataPoints)*TimeInterval
		
        self.Charge = np.zeros(self.DataPoints)
        self.Voltage = np.zeros(self.DataPoints)
        self.Current = np.zeros(self.DataPoints)
        self.PowerOut = np.zeros(self.DataPoints)
 
    @jit
    def DrainCaps(self,CurrentCharge,Current,TimeInterval):
        Charge = CurrentCharge - Current*TimeInterval
        return(Charge)
	
    @jit
    def calc_Voltage(self,Charge):
        Voltage = Charge / self.Capacitance
        return(Voltage)
    
    @jit    
    def calc_Charge(self,Voltage):
        Charge = Voltage * self.Capacitance
        return(Charge)

    def calc_Current(self):
        self.Current[1:] = -np.diff(self.Voltage)*self.Capacitance/self.TimeInterval
    
    def calc_PowerOut(self):
        self.PowerOut = self.Voltage * self.Current

    #plotting
    def plot_VoltageCharge(self):
        plt.plot(self.Charge,self.Voltage)
        plt.ylabel('Voltage')
        plt.xlabel('Charge')
        plt.title('SuperCaps')
        plt.show()
        
    def plot_ChargeTime(self):
        plt.plot(self.TimeEllapsed,self.Charge)
        plt.xlabel('Time')
        plt.ylabel('Charge')
        plt.title('SuperCaps')
        plt.show()
        
    def plot_CurrentTime(self):
        plt.plot(self.TimeEllapsed,self.Current)
        plt.xlabel('Times (S)')
        plt.ylabel('Current (Amps)')
        plt.show()