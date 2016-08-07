# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

class Motor_c:
    """ Class containing model of Motor """
    
    DataPoints = 0
    TimeInterval = 0
    SimulationTime = 0
    
    #Constants
    VelocityConstant = 0 #rad/Vs
    WindingResistance = 0 #winding resistence ohms
    TorqueConstant = 0 #Nm/A
    BackEMFConstant = 0 #Vs/rad
    MotorConstant = 0 #Nm/sqrt(W)   (W == Watts)
    
    MaxVoltage = 0 #max nominal voltage (V)
    MaxSpeed = 0 #max no load speed (rad/s)

    MaxCurrent = 0 #max short term current (A) (stop coils from melting)
    NoLoadCurrent = 0 #motor current at full speed with no load (A)    
    
    TorqueLoss = 0 #torque loses from motor (Nm)
    MotorInertia = 0 #inertia of motor armature (kg m^2)
    
    #Variable arrays
    Torque = ''
    PowerIn = ''
    PowerOut = ''
    Efficiency = ''
    Voltage = ''
    Current = ''
    Acceleration = ''
    Speed = ''    
    TimeEllapsed = ''
    
    
    ## FUNCTIONS ##
    def __init__(self,SimulationTime,TimeInterval):
        #class constructor
        print('Motor Object Created')
        
        self.SimulationTime = SimulationTime
        self.TimeInterval = TimeInterval
        self.DataPoints = math.floor(SimulationTime/TimeInterval)      
        
        self.TimeEllapsed = np.arange(self.DataPoints)*TimeInterval
        self.TimeEllapsed.reshape(self.DataPoints)        
        
        #Allocate RAM)
        self.Torque = np.zeros(self.DataPoints)
        self.PowerIn = np.zeros(self.DataPoints)
        self.PowerOut = np.zeros(self.DataPoints)
        self.Voltage = np.zeros(self.DataPoints)
        self.Current = np.zeros(self.DataPoints)
        self.Acceleration = np.zeros(self.DataPoints)
        self.Speed = np.zeros(self.DataPoints)
        self.Efficiency = np.zeros(self.DataPoints)
        
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
                    self.VelocityConstant = 1 / self.MotorConstant / math.sqrt(self.WindingResistance)
                else:
                    print('Unable to Calculate Motor Constants')
            else:
                print('Unable to Calculate Motor Constants')
        
        if self.BackEMFConstant == 0:
            self.BackEMFConstant = 1/self.VelocityConstant
        
        if self.TorqueConstant == 0:
            self.TorqueConstant = self.BackEMFConstant

        if self.MotorConstant == 0:
            if self.WindingResistance != 0:
                self.MotorConstant = self.VelocityConstant / math.sqrt(self.WindingResistance)
            else:
                print('Unable to Calculate Motor Constants')
        
        if self.WindingResistance == 0:
            if self.MotorConstant != 0:
                self.WindingResistance = math.pow((self.VelocityConstant / self.MotorConstant),2)
            else:
                print('Unable to Calculate Motor Constants')
        if self.TorqueLoss == 0:
            if (self.MaxSpeed != 0) & (self.MaxVoltage != 0):
                self.TorqueLoss = -1*self.BackEMFConstant*self.TorqueConstant/self.WindingResistance*self.MaxSpeed + self.MaxVoltage*self.TorqueConstant/self.WindingResistance
            elif self.NoLoadCurrent != 0:
                self.TorqueLoss = self.TorqueConstant * self.NoLoadCurrent
            else:
                print('Unable to Calculate Motor Losses')
            
    def calc_Torque(self,Index):
        #motor Torque Speed Curve
        self.Torque[Index] = -1*self.BackEMFConstant*self.TorqueConstant/self.WindingResistance*self.Speed[Index]+self.Voltage[Index]*self.TorqueConstant/self.WindingResistance-self.TorqueLoss
        return(self.Torque[Index])        
    
    def calc_Current(self,Index):
        #motor torque constant
        self.Current[Index] = (self.Torque[Index]+self.TorqueLoss)/self.TorqueConstant;
        return(self.Current[Index])
        
    def calc_Efficiency(self,Index):
        #Efficiency
        self.PowerIn[Index] = self.Voltage[Index] * self.Current[Index]
        self.PowerOut[Index] = self.Torque[Index]*self.Speed[Index]
        self.Efficiency[Index] = self.PowerOut[Index] / self.PowerIn[Index]
        return(self.Efficiency[Index])

        
    #Unit Conversions
    def rmp_per_V_2_rad_per_Vs(self,rmp_per_V):
        rad_per_Vs = rmp_per_V / 60 * 2 * np.pi
        return(rad_per_Vs)
    
    def rmp_2_rad_per_s(self,rmp):
        rad_per_s = rmp / 60 * 2 * np.pi
        return(rad_per_s)


    #Plotting
    def plot_TorqueSpeed(self):
        plt.plot( self.Speed, self.Torque )
        plt.show()

    def plot_EfficiencySpeed(self):
        plt.plot( self.Speed, self.Efficiency )
        plt.show()
        
    def plot_PowerSpeed(self):
        plt.plot( self.Speed, self.PowerIn , self.Speed , self.PowerOut )
        plt.show()
    
    def plot_SpeedTime(self):
        plt.plot(self.TimeEllapsed, self.Speed)
        plt.show()
        
