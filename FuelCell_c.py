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
    def calc_StackVoltage(self,Index):
        # Tafel Coefficient, in V
        self.A = self.RealGas * self.StackTemperature / 2 / self.Alpha / self.Faraday
        
        if Index != 0:
            if self.StackCurrent[Index-1] > 0:
                b = self.A*math.log(self.StackCurrent[Index-1]/(self.CellArea*self.ExchangeCurrentDensity/1000))
                if b > 0: 
                    self.StackVoltage[Index] = self.CellNumber * ( self.CellOCVoltage - self.StackCurrent[Index-1]*self.CellResistance/self.CellArea-b )
                else:
                    self.StackVoltage[Index] = self.CellNumber * ( self.CellOCVoltage - self.StackCurrent[Index-1]*self.CellResistance/self.CellArea )
            else:
                self.StackVoltage[Index] = self.CellNumber * ( self.CellOCVoltage )
        else:
            self.StackVoltage[Index] = self.CellNumber * ( self.CellOCVoltage )
            
            
        return(self.StackVoltage[Index])
        
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