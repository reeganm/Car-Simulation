# -*- coding: utf-8 -*-

import numpy as np
import math

from Motor_c import Motor_c
from FuelCell_c import FuelCell_c
from Track_c import Track_c
from Car_c import Car_c
from SuperCapacitor_c import SuperCapacitor_c

SimulationTime = 240 #seconds
TimeInterval = 0.0001 #time step/integration interval #make a lot smaller than total inertia to decrease motor integration error
TrackLength = 100 #100 meter track

DataPoints = math.floor(SimulationTime/TimeInterval)


## MOTOR ##

#create motor object
motor = Motor_c(SimulationTime,TimeInterval)
#Set Motor Constants
motor.VelocityConstant = motor.rmp_per_V_2_rad_per_Vs(250)
motor.WindingResistance = 0.08 #ohms
motor.NoLoadCurrent = 2
#motor.MaxSpeed = motor.rmp_2_rad_per_s(11100)
motor.Inertia = 0.0000 #kg m2
#calculate other motor parameters
motor.calc_MissingMotorConstants()


## FUELCELL ##

#create fuelcell object
fuelcell = FuelCell_c(SimulationTime,TimeInterval)
#Set FuelCell parameters
fuelcell.CellNumber = 46
fuelcell.CellArea = 250 #cm2
fuelcell.CellResistance = 0.62
fuelcell.Alpha = 0.6
fuelcell.ExchangeCurrentDensity = 0.04
fuelcell.CellOCVoltage = 1.005
fuelcell.DiodeVoltageDrop = 0.7


## TRACK ##

#create track object
track = Track_c(SimulationTime,TimeInterval)
#set track parameters
track.Incline = 0 #deg
track.RelativeHumidity = 50 #%
track.Temperature = 30 #Celcius
track.AirPressure = 101 #kPa
track.calc_AirDensity()


## SUPERCAPS ##

#create super capacitor object
supercaps = SuperCapacitor_c(SimulationTime,TimeInterval)
#set super capacitor parameters


## CAR ##

#create car object
car = Car_c(SimulationTime,TimeInterval)
#set car parameters
car.GearRatio = 28.0
car.GearEfficiency = 0.9
car.GearInertia = 0
car.Mass = 310 #kg
car.WheelDiameter = 0.56
car.WheelInertia = 0
car.BearingResistance = 10 #N
car.RollingResistanceCoefficient = 0.01
car.AreodynamicDragCoefficient = 0.3
car.FrontalArea = 1.2*1.67

#### SIMULATION ####

#initial conditions
motor.Voltage[0] = fuelcell.calc_StackVoltage(0)
motor.Torque[0] = motor.calc_Torque(0)
motor.calc_Current(0)
fuelcell.StackCurrent[0] = motor.Current[0]

for n in range(1,DataPoints):

    car.Speed[n] = car.Speed[n-1] + car.Acceleration[n-1] * TimeInterval
    car.calc_AirDrag(track.AirDensity,n)

    motor.Speed[n] = car.Speed[n] * car.GearRatio / car.WheelDiameter * 2
    motor.Voltage[n] = fuelcell.calc_StackVoltage(n)
    motor.calc_Torque(n)
    
    #fuelcell.StackCurrent[n] = motor.calc_Current(n)
    motor.calc_Efficiency(n)


    car.Acceleration[n] = (motor.Torque[n]*car.WheelDiameter/2*car.GearRatio*car.GearEfficiency-car.Mass*math.sin(track.Incline/180*np.pi)*9.81-car.AirDrag-car.RollingResistanceCoefficient-car.BearingResistance) / (car.Mass+car.WheelInertia) / (1 + ((car.GearInertia + math.pow(car.GearRatio,2)*car.GearRatio*motor.MotorInertia ) / (car.Mass + car.WheelInertia) )  )
    if car.Acceleration[n] < 0:
        if car.Speed[n] <= 0:
            car.Acceleration[n] = 0

    car.DistanceTravelled[n] = car.DistanceTravelled[n-1] + car.Speed[n-1]* TimeInterval

    motor.Acceleration[n] = car.Acceleration[n] / car.WheelDiameter / car.GearRatio


motor.plot_TorqueSpeed()
motor.plot_PowerSpeed()
motor.plot_EfficiencySpeed()
motor.plot_SpeedTime()