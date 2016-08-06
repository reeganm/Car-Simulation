# -*- coding: utf-8 -*-

import numpy as np

class Motor_c:
    """ Class containing model of Motor """
    
    #Constants
    VelocityConstant = 0 #rad/Vs
    WindingResistance = 0 #winding resistence ohms
    TorqueConstant = 0 #Nm/A
    BackEMFConstant = 0 #Vs/rad
    MotorConstant = 0 #Nm/sqrt(W)   (W == Watts)
    
    MaxVoltage = 0 #max nominal voltage (V)
    MaxSpeed = 0 #max no load speed (rad/s)

    MaxCurrent = 0 #max short term current (A) (stop coils from melting)
    
    TorqueLoss = 0 #torque loses from motor
    
    #Variable arrays
    MotorTorque = ''
    MotorPowerIn = ''
    MotorPowerOut = ''
    MotorEfficiency = ''
    MotorVoltage = ''
    MotorCurrent = ''
    MotorSpeed = ''    
    
    ## FUNCTIONS ##
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
    
    #Motor Equations
    def calc_MissingMotorConstants(self):
        # TorqueConstant = BackEMFConstant = 1/Kv
        if self.VelocityConstant == 0:
            if self.BackEMFConstant != 0:
                self.VelocityConstant = 1 / self.BackEMFConstant
            elif self.TorqueConstant != 0:
                self.VelocityConstant = 1 / self.TorqueConstant
            elif self.MotorConstant != 0:
                if self.WindingResistance != 0:
                    self.VelocityConstant = 1 / self.MotorConstant / (self.WindingResistance)^(1/2)
                else:
                    print('Unable to Calculate Motor Constants')
            else:
                print('Unable to Calculate Motor Constants')
        
        if self.BackEMFConstant == 0:
            self.BackEMFConstant = 1/self.VelocityConstant
        
        if self.TorqueConstant == 0:
            self.BackEMFConstant = self.TorqueConstant

        if self.MotorConstant == 0:
            if self.WindingResistance != 0:
                self.MotorConstant = self.VelocityConstant / (self.WindingResistance)^(1/2)
            else:
                print('Unable to Calculate Motor Constants')
        
        if self.WindingResistance == 0:
            if self.MotorConstant != 0:
                self.WindingResistance = (self.VelocityConstant / self.MotorConstant)^(2)
            else:
                print('Unable to Calculate Motor Constants')
        if self.TorqueLoss == 0:
            if self.MaxSpeed != 0:
                if self.MaxVoltage != 0:
                    self.TorqueLoss = -1*self.BackEMFConstant*self.TorqueConstant/self.WindingResistance*self.MaxSpeed + self.MaxVoltage*self.TorqueConstant/self.WindingResistance
                else:
                    print('Unable to Calculate Motor Losses')
            else:
                print('Unable to Calculate Motor Losses')
            
    def calc_MotorTorque(self,Index):
        #motor Torque Speed Curve
        self.MotorTorque[Index] = -1*self.BackEMFConstant*self.TorqueConstant[Index]/self.WindingResistance*self.MotorSpeed[Index]+self.MotorVoltage[Index]*self.TorqueConstant/self.WindingResistance
        return(self.MotorTorque[Index])        
    
    def calc_MotorCurrent(self,Index):
        #motor torque constant
        self.MotorCurrent[Index] = self.MotorTorque[Index]/self.TorqueConstant;
        return(self.MotorCurrent[Index])
        
    def calc_MotorPowerIn(self,Index):
        #calculate electric power given to motor
        self.MotorPowerIn[Index] = self.MotorVoltage[Index] * self.MotorCurrent[Index]
        return(self.MotorPowerIn[Index])

    def calc_MotorPowerOut(self,Index):    
        #power output
        self.MotorPowerOut[Index] = self.MotorTorque[Index]*self.MotorSpeed[Index]
        return(self.MotorPowerOut[Index])     
        
    #Unit Conversions
    def rmp_per_V_2_rad_per_Vs(rmp_per_V):
        rad_per_Vs = rmp_per_V / 60 * 2 * 3.141592654
        return(rad_per_Vs)
    
    def rmp_2_rad_per_s(rmp):
        rad_per_s = rmp / 60 * 2 * 3.141592654
        return(rad_per_s)
        