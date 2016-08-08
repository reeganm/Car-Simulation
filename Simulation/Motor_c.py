# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
from numba import jit


class Motor_c:
    """ Class containing model of Motor """
    
    DataPoints = 0
    TimeInterval = 0
    SimulationTime = 0
    
    # Motor Constants
    # https://en.wikipedia.org/wiki/Motor_constants
    # http://learningrc.com/motor-kv/
    VelocityConstant = 0 #rad/Vs
    TorqueConstant = 0 #Nm/A
    BackEMFConstant = 0 #Vs/rad
    MotorConstant = 0 #Nm/sqrt(W)   (W == Watts)
    
    #winding resistence ohms 
    WindingResistance = 0    
    
    ## These Parameters are used to calculate motor torque loss
    #max nominal voltage reported by manufacturer
    MaxVoltage = 0 # V
    #max no load speed at that voltage
    MaxSpeed = 0 # rad/s
    #motor current at full speed with no load
    NoLoadCurrent = 0 
    #torque losses from motor (Nm) "iddle torque"
    TorqueLoss = 0 
    
    #Some controllers limit motor current set that here
    MaxCurrent = 1000 
        
    
    #inertia of motor armature
    MotorInertia = 0 # kg m^2
    
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
        #make time array for plotting
        self.TimeEllapsed = np.arange(self.DataPoints)*TimeInterval
        
        #Allocate RAM
        self.Torque = np.zeros(self.DataPoints)
        self.PowerIn = np.zeros(self.DataPoints)
        self.PowerOut = np.zeros(self.DataPoints)
        self.Voltage = np.zeros(self.DataPoints)
        self.Current = np.zeros(self.DataPoints)
        self.Acceleration = np.zeros(self.DataPoints)
        self.Speed = np.zeros(self.DataPoints)
        self.Efficiency = np.zeros(self.DataPoints)
    
    @jit
    #Motor Equations
    def calc_MissingMotorConstants(self):
        # TorqueConstant = BackEMFConstant = 1/Kv = MotorContant * sqrt(WindingResistance)
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
                
        #torque loss equals position on torque speed curve at no load idle speed
        if self.TorqueLoss == 0:
            if (self.MaxSpeed != 0) & (self.MaxVoltage != 0):
                self.TorqueLoss = -1*self.BackEMFConstant*self.TorqueConstant/self.WindingResistance*self.MaxSpeed + self.MaxVoltage*self.TorqueConstant/self.WindingResistance
            elif self.NoLoadCurrent != 0:
                self.TorqueLoss = self.TorqueConstant * self.NoLoadCurrent
            else:
                print('Unable to Calculate Motor Losses')
    
    @jit    
    def calc_Torque(self,Index):
        #motor Torque Speed Curve
        self.Torque[Index] = -1*self.BackEMFConstant*self.TorqueConstant/self.WindingResistance*self.Speed[Index]+self.Voltage[Index]*self.TorqueConstant/self.WindingResistance-self.TorqueLoss
        return(self.Torque[Index])        
    
    @jit
    def calc_Current(self,Index):
        #motor torque constant
        self.Current[Index] = (self.Torque[Index]+self.TorqueLoss)/self.TorqueConstant;
        #limit motor current to max current
        if self.Current[Index] > self.MaxCurrent:
            self.Current[Index] = self.MaxCurrent
            #need to recalculate torque based off of current limit
            self.Torque[Index] = self.Current[Index]*self.TorqueConstant-self.TorqueLoss
        else:
            self.MaxCurrent = 60
        return(self.Current[Index])
    
    @jit
    def calc_Efficiency(self):
        ## Efficiency ##
        #electrical power in
        self.PowerIn = self.Voltage * self.Current
        #mechanical power out
        self.PowerOut = self.Torque*self.Speed
        # power out / power in
        self.Efficiency = self.PowerOut / self.PowerIn

        
    #Unit Conversions
    def rmp_per_V_2_rad_per_Vs(self,rmp_per_V):
        rad_per_Vs = rmp_per_V / 60 * 2 * np.pi
        return(rad_per_Vs)
    
    def rmp_2_rad_per_s(self,rmp):
        rad_per_s = rmp / 60 * 2 * np.pi
        return(rad_per_s)

    def gcm2_2_kgm2(self,gcm2):
        kgm2 = gcm2 / 10000000
        return(kgm2)

    #Plotting
    def plot_TorqueSpeed(self):
        plt.plot( self.Speed, self.Torque )
        plt.xlabel('Speed (rad/s)')
        plt.ylabel('Torque (Nm')
        plt.title('Motor')
        plt.show()

    def plot_EfficiencySpeed(self):
        plt.plot( self.Speed, self.Efficiency )
        plt.xlabel('Speed (rad/s)')
        plt.ylabel('Efficiency')
        plt.title('Motor')
        plt.show()
        
    def plot_PowerSpeed(self):
        plt.plot( self.Speed, self.PowerIn , self.Speed , self.PowerOut )
        plt.xlabel('Speed (rad/s)')
        plt.ylabel('Power (W)')
        plt.title('Motor')
        plt.show()
    
    def plot_SpeedTime(self):
        plt.plot(self.TimeEllapsed, self.Speed)
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (rad/s)')
        plt.title('Motor')
        plt.show()
    
    def plot_TorqueTime(self):
        plt.plot(self.TimeEllapsed,self.Torque)
        plt.xlabel('Time (s)')
        plt.ylabel('Torque (Nm)')
        plt.title('Motor')
        plt.show()

    def plot_CurrentTime(self):
        plt.plot(self.TimeEllapsed, self.Current)
        plt.xlabel('Time (s)')
        plt.ylabel('Current (A)')
        plt.title('Motor')
        plt.show()
        
    def plot_VoltageTime(self):
        plt.plot(self.TimeEllapsed, self.Voltage)
        plt.xlabel('Time (s)')
        plt.ylabel('Voltage (V)')
        plt.title('Motor')
        plt.show()
        