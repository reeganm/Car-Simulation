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
    StackPowerOut = ''
    StackEnergyProduced = ''
    StackEnergyConsumed = ''
    
    CurveI = []
    CurveV = []
    
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
        self.StackPowerOut = np.zeros(self.DataPoints)        
        self.StackEnergyProduced = np.zeros(self.DataPoints)
        self.StackEnergyConsumed = np.zeros(self.DataPoints)
        
        self.CurveI = np.arange(0 , 120, 0.1)
        self.CurveV = np.zeros(len(self.CurveI))
		
  
    @jit
    def build_VoltageCurrentCurve(self):
        A = self.RealGas * self.StackTemperature / 2 / self.Alpha / self.Faraday

        for i in range(len(self.CurveV)):
            current = self.CurveI[i]
            b = current / ( self.CellArea * self.ExchangeCurrentDensity / 1000)
            if b > 0:
                self.CurveV[i] = self.CellNumber * (self.CellOCVoltage - current*self.CellResistance/self.CellArea - A*np.log(b))
            else:
                self.CurveV[i] = self.CellNumber * (self.CellOCVoltage - current*self.CellResistance/self.CellArea)
                
                
    @jit
    def calc_StackCurrent(self,Voltage):
        #calculate available current at specified voltage
        index = (self.CurveV > Voltage).argmin()
        #interpolation
        V1 = self.CurveV[index-1]
        V2 = self.CurveV[index]
        
        I1 = self.CurveI[index-1]
        I2 = self.CurveI[index]
        
        Current = I1 + (I2-I1)/(V2-V1)*(Voltage-V1)
        
        if Current < 0:
            Current = 0
            
        if Current > 120:
            print('Warning: FuelCell Current Outside of Interpolation Range')
            
        return(Current)
    
    @jit    
    def calc_StackVoltage(self,Current):
        #calculates stack voltage based off of current draw
        
        A = self.RealGas * self.StackTemperature / 2 / self.Alpha / self.Faraday

        b = Current / ( self.CellArea * self.ExchangeCurrentDensity / 1000)
        if b > 0:
            Voltage = self.CellNumber * (self.CellOCVoltage - Current*self.CellResistance/self.CellArea - A*np.log(b))
        else:
            Voltage = self.CellNumber * (self.CellOCVoltage - Current*self.CellResistance/self.CellArea)
        return(Voltage)        
    
    @jit
    def calc_StackEfficiency(self):
        self.StackEfficiency = self.StackVoltage / self.CellNumber / self.TheoreticalCellVoltage
        
    @jit 
    def calc_StackPowerOut(self):
        self.StackPowerOut = self.StackVoltage * self.StackCurrent
    
    @jit 
    def calc_StackEnergyProduced(self,Voltage,Current,Time):
        StackEnergyProduced = Voltage*Current*Time
        return(StackEnergyProduced)
        
    @jit 
    def calc_StackEnergyConsumed(self,Current,Time):
        StackEnergyProduced = self.TheoreticalCellVoltage*self.CellNumber*Current*Time
        return(StackEnergyProduced)
    
    #Plotting
    def plot_FCCurve(self):
        plt.plot( self.CurveI, self.CurveV )
        plt.xlabel('Stack Current')
        plt.ylabel('Stack Voltage')
        plt.title('Fuel Cell curve')
        plt.show()
        
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
        
    def plot_StackCurrentTime(self):
        plt.plot(self.TimeEllapsed,self.StackCurrent)
        plt.xlabel('Time')
        plt.ylabel('Stack Current')
        plt.title('Fuel Cell')
        plt.show()
        