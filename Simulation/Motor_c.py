# -*- coding: utf-8 -*-

import numpy as np

class Motor_c:
    """ Class containing model of Motor """
    
    #Constants
    VoltageConstant = 0 #voltage constant
    WindingEfficiency = 0 #winding efficiency
    WindingResistance = 0 #winding resistence ohms
    MotorMaxCurrent = 0 #max short term current
    MotorMaxSpeed = 0 #max motor speed
    IdleCurrent = 0 #motor idling current
    TorqueConstant = 0 #torque constant
    
    #Variable arrays
    MotorTorque = ''
    MotorPowerIn = ''
    MotorPowerOut = ''
    MotorEfficiency = ''
    MotorVoltage = ''
    MotorCurrent = ''
    MotorSpeed = ''    
    
    #equations
    def calc_TorqueConstant(self):
        #calculate torque constant
        if self.WindingEfficiency == 0:
            print('Warning: Winding Efficiency Not Set')
        if self.VoltageConstant == 0:
            print('Warning: Voltage Constant Not Set')
            
        TorqueConstant=self.WindingEfficiency/(self.VoltageConstant*2*3.141592654/60)
        return(TorqueConstant)
    
    def calc_MotorTorque(self,Index):
        #calculate motor torque from current
        if self.TorqueConstant[Index] == 0:
            print('Warning Torque Constant Not Set')
        if self.IddleCurrent[Index] == 0:
            print('Warning Iddle Current Not Set')

        self.MotorTorque[Index] = self.TorqueConstant*(self.MotorCurrent[Index]-self.IdleCurrent)
        return(self.MotorTorque[Index])        
        
    def calc_MotorPowerIn(self,Index):
        #calculate power given to motor
        self.MotorPowerIn[Index] = self.MotorVoltage[Index] * self.MotorCurrent[Index]
        return(self.MotorPowerIn[Index])

    def calc_MotorPowerOut(self,Index):    
        #power output
        self.MotorPowerOut[Index] = self.MotorTorque[Index]*self.MotorSpeed[Index]
        return(self.MotorPowerOut[Index])
        
    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('Motor Object Created')
        
        #Allocate RAM
        DataPoints = SimulationTime/TimeInterval
        DataPoints = int(DataPoints)        
        self.MotorTorque = np.zeros((DataPoints,1))
        self.MotorPowerIn = np.zeros((DataPoints,1))
        self.MotorPowerOut = np.zeros((DataPoints,1))
        self.MotorVoltage = np.zeros((DataPoints,1))
        self.MotorCurrent = np.zeros((DataPoints,1))
        self.MotorSpeed = np.zeros((DataPoints,1))
        self.MotorEfficiency = np.zeros((DataPoints,1))
        