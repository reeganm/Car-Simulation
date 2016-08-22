# -*- coding: utf-8 -*-

import numpy as np
import math
from numba import jit
import matplotlib.pyplot as plt


#### SIMULATION ####
@jit
def run_Simulation(motor,fuelcell,car,track,supercap,DataPoints,TimeInterval):
    #Starting Simulation
    
    ## initial conditions ##
    
    fuelcell.StackCurrent[0] = fuelcell.AuxCurrent
    fuelcell.StackVoltage[0] = fuelcell.calc_StackVoltage(fuelcell.StackCurrent[0])
    
    supercap.Charge[0] = supercap.calc_Charge(fuelcell.StackVoltage[0]-fuelcell.DiodeVoltageDrop)
    supercap.Voltage[0] = supercap.calc_Voltage(supercap.Charge[0])
    
    
    ## Solve Differential Equations Numerically in for Loop ##
    
    for n in range(1,DataPoints):
        car.Speed[n] = car.Speed[n-1] + car.Acceleration[n-1]*TimeInterval
        motor.Speed[n] = car.Speed[n] / car.WheelDiameter * 2 * car.GearRatio
        
        motor.Torque[n],motor.Current[n] = motor.calc_MotorTorqueCurrent(motor.Voltage[n-1],motor.Speed[n-1])
        
        #motor current drains caps
        supercap.Charge[n] = supercap.DrainCaps(supercap.Charge[n-1],motor.Current[n],TimeInterval)
                
        #fuel cell supplies caps
        fuelcell.StackCurrent[n] = fuelcell.calc_StackCurrent(fuelcell.StackVoltage[n-1])
        supercap.Charge[n] = supercap.DrainCaps(supercap.Charge[n],-1*fuelcell.StackCurrent[n],TimeInterval)
        
        #super cap voltage changes based on charge
        supercap.Voltage[n] = supercap.calc_Voltage(supercap.Charge[n])
        #fuelcell voltage from cap voltage
        fuelcell.StackVoltage[n] = supercap.Voltage[n] + fuelcell.DiodeVoltageDrop
        
        motor.Voltage[n] = supercap.Voltage[n]
        
        car.AirDrag[n] = car.calc_AirDrag(track.AirDensity,car.Speed[n])
        car.Acceleration[n] = (motor.Torque[n]*car.WheelDiameter/2*car.GearRatio*car.GearEfficiency-car.Mass*math.sin(track.Incline/180*np.pi)*9.81-car.AirDrag[n]-car.RollingResistanceCoefficient-car.BearingResistance) / (car.Mass+car.WheelInertia) / (1 + ((car.GearInertia + math.pow(car.GearRatio,2)*car.GearRatio*motor.MotorInertia ) / (car.Mass + car.WheelInertia) )  )
        #stop car from moving backwards if oposing forces are too high
        if car.Acceleration[n] < 0:
            car.Acceleration[n] = 0    
        
        car.DistanceTravelled[n] = car.DistanceTravelled[n-1] + car.Speed[n-1]*TimeInterval
        
        
        
    ## These calculations can be vectorized instead of being in for loop ##
    
    #calculate motor efficiency curve
    motor.calc_Efficiency()
    #calculate fuelcell efficiency curve
    fuelcell.calc_StackEfficiency()

    #instantaneous driving efficiency -> power into motor vs speed
    car.Milage = car.Speed  / (fuelcell.CellNumber*fuelcell.TheoreticalCellVoltage*fuelcell.StackCurrent / 1000) * 33.7 #  mile/per gallon (33.7 kWhr in 1 Gallon. 3.6 MJ in one kWhr)
    
    