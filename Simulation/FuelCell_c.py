# -*- coding: utf-8 -*-

import numpy as np
import math
from numba import jit
import matplotlib.pyplot as plt


class FuelCell_c:
    """ Class Constaining Model of FuelCell """
    
    #Constants
    CellNumber = 0
    CellArea = 0
    CellResistance = 0
    Alpha = 0
    ExchangeCurrentDensity = 0
    CellOCVoltage = 0
    DiodeVoltageDrop = 0
    TheoreticalCellVoltage = 1.48

    Faraday = 96485 #Faraday C/mol
    RealGas = 8.314462 #J/molK
    
    AuxCurrent = 0    
    StackTemperature = 300
    
    #Variable Arrays
    StackVoltage = ''
    StackCurrent = ''
    StackEfficiency = ''
    
    
    DataPoints = ''
    SimulationTime = ''
    TimeInterval = ''
    TimeEllapsed = ''
    
    A = 0

    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('FuelCell Object Created')
        
        self.SimulationTime = SimulationTime
        self.TimeInterval = TimeInterval
        
        self.DataPoints = math.floor(SimulationTime/TimeInterval)
        self.TimeEllapsed = np.arange(self.DataPoints)*TimeInterval
        
        self.StackVoltage = np.zeros(self.DataPoints)
        self.StackCurrent = np.zeros(self.DataPoints)
        self.StackEfficiency = np.zeros(self.DataPoints)
        
        
	@jit
	def calc_StackCurrent(self,Current):
		#calculate available current at specified voltage
		
		return(Current)
		
    @jit
    def calc_StackEfficiency(self):
        self.StackEfficiency = self.StackVoltage / self.CellNumber / self.TheoreticalCellVoltage
        
    #Plotting
    def plot_StackVoltageCurrent(self):
        plt.plot( self.StackCurrent, self.StackVoltage )
        plt.xlabel('Stack Current (A)')
        plt.ylabel('Stack Voltage (V)')
        plt.title('FuelCell')
        plt.show()
        
    def plot_StackEfficiency(self):
        plt.plot( self.TimeEllapsed , self.StackEfficiency )    
        plt.xlabel('Time (s)')
        plt.ylabel('Stack Efficiency')
        plt.title('FuelCell')
        plt.show()