# -*- coding: utf-8 -*-

import numpy as np
import math

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
    
    #Variable Arrays
    StackVoltage = ''
    StackCurrent = ''
    StackTemperature = ''
    
    
    DataPoints = ''
    SimulationTime = ''
    TimeInterval = ''
    
    A = 0

    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('FuelCell Object Created')
        
        self.SimulationTime = SimulationTime
        self.TimeInterval = TimeInterval
        
        self.DataPoints = math.floor(SimulationTime/TimeInterval)
        
        self.StackVoltage = np.zeros(self.DataPoints)
        self.StackCurrent = np.zeros(self.DataPoints)
        self.StackTemperature = np.zeros(self.DataPoints)

        self.A = self.RealGas * self.StackTemperature / 2 / self.Alpha / self.Faraday
        
        
    def calc_StackVoltage(self,Index):
        # Tafel Coefficient, in V
        A = self.A
        if Index != 0:
            if self.StackCurrent[Index-1] > 0:
                b = A*math.log(self.StackCurrent[Index-1]/(self.CellArea/(self.ExchangeCurrentDensity/1000))
                if b > 0: #sum fixes b being numpy type error
                    self.StackVoltage[Index] = self.CellNumber * ( self.CellOCVoltage - self.StackCurrent[Index-1]*self.CellResistance/self.CellArea-b )
                else:
                    self.StackVoltage[Index] = self.CellNumber * ( self.CellOCVoltage - self.StackCurrent[Index-1]*self.CellResistance/self.CellArea )
            else:
                self.StackVoltage[Index] = self.CellNumber * ( self.CellOCVoltage )
        else:
            self.StackVoltage[Index] = self.CellNumber * ( self.CellOCVoltage )
            
            
        return(self.StackVoltage[Index])