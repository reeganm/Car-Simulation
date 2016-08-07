# -*- coding: utf-8 -*-

import numpy as np
import math
from numba import jit


#### SIMULATION ####
@jit
def run_Simulation(motor,fuelcell,car,track,supercaps,DataPoints,TimeInterval):
    #Starting Simulation
    
    ## initial conditions ##
    #motor voltage is a function of Stack Voltage which is a function of current which is zero
    motor.Voltage[0] = fuelcell.calc_StackVoltage(0)-fuelcell.DiodeVoltageDrop 
    # torque is a function of voltage and speed
    motor.Torque[0] = motor.calc_Torque(0) 
    #motor current is a function of torque
    motor.calc_Current(0) 
    #fuelcell current is total current
    fuelcell.StackCurrent[0] = motor.Current[0] + fuelcell.AuxCurrent
    
    ## Solve Differential Equations Numerically in for Loop ##
    for n in range(1,DataPoints):
        #update speed using previous acceleration
        car.Speed[n] = car.Speed[n-1] + car.Acceleration[n-1] * TimeInterval 
        # calculate air drag using new speed
        car.calc_AirDrag(track.AirDensity,n) 
        #calculate motor speed from car speed
        motor.Speed[n] = car.Speed[n] * car.GearRatio / car.WheelDiameter * 2

        #motor voltage is a function of stack voltage which is a function of stack current
        motor.Voltage[n] = fuelcell.calc_StackVoltage(n) - fuelcell.DiodeVoltageDrop 
        #torque is a function of speed and voltage
        motor.calc_Torque(n) 
        
        # motor current is a function of torque
        motor.calc_Current(n) 
        fuelcell.StackCurrent[n] = motor.calc_Current(n) + fuelcell.AuxCurrent
        
        #calculate car acceleration from ugly FBD
        car.Acceleration[n] = (motor.Torque[n]*car.WheelDiameter/2*car.GearRatio*car.GearEfficiency-car.Mass*math.sin(track.Incline/180*np.pi)*9.81-car.AirDrag-car.RollingResistanceCoefficient-car.BearingResistance) / (car.Mass+car.WheelInertia) / (1 + ((car.GearInertia + math.pow(car.GearRatio,2)*car.GearRatio*motor.MotorInertia ) / (car.Mass + car.WheelInertia) )  )
        #stop car from moving backwards if oposing forces are too high
        if car.Acceleration[n] < 0:
            if car.Speed[n] <= 0:
                car.Acceleration[n] = 0
                
        #calculate motor acceleration from car acceleration
        motor.Acceleration[n] = car.Acceleration[n] / car.WheelDiameter / car.GearRatio 
        
        #update distance travelled from speed
        car.DistanceTravelled[n] = car.DistanceTravelled[n-1] + car.Speed[n-1]* TimeInterval
    
    ## These calculations can be vectorized instead of being in for loop ##
    
    #calculate motor efficiency curve
    motor.calc_Efficiency()
    #calculate fuelcell efficiency curve
    fuelcell.calc_StackEfficiency()

    