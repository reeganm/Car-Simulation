# -*- coding: utf-8 -*-

import numpy as np
import math
from numba import jit
import matplotlib.pyplot as plt

from Motor_c import Motor_c
from FuelCell_c import FuelCell_c
from Track_c import Track_c
from Car_c import Car_c
from SuperCapacitor_c import SuperCapacitor_c
import Simulation


SimulationTime = 600 #seconds
TimeInterval = 0.01 #time step/integration interval #make a lot smaller than total inertia to decrease motor speed integration error
TrackLength = 4500 # meters

DataPoints = math.floor(SimulationTime/TimeInterval)


#### MOTOR ####

#create motor object
motor = Motor_c(SimulationTime,TimeInterval)

## Set Motor Constants ##
#Velocity constant, Torque constant, BackEMFConstant are all related only need one https://en.wikipedia.org/wiki/Motor_constants
motor.VelocityConstant = motor.rpm_per_V_2_rad_per_Vs(250) # rad/Vs
motor.WindingResistance = 0.08
motor.NoLoadCurrent = 2
#inertia of motor armature (not too important can be set to zero)
motor.Inertia = 0 
#set to limit motor current (ie some controllers limit current)  default is 1000 Amps
motor.MaxCurrent = 60 #Amp
#calculate other motor parameters
motor.calc_MissingMotorConstants()
motor.plot_TorqueSpeedCurve()


## FUELCELL ##

#create fuelcell object
fuelcell = FuelCell_c(SimulationTime,TimeInterval)
#Set FuelCell parameters
fuelcell.CellNumber = 46
fuelcell.CellArea = 145 #cm2
fuelcell.CellResistance = 0.36
fuelcell.Alpha = 0.45
fuelcell.ExchangeCurrentDensity = 0.04
fuelcell.CellOCVoltage = 1.02 #open circuit voltage
fuelcell.DiodeVoltageDrop = 0.5
fuelcell.AuxCurrent = 2 #current consumed by controllers, fans etc (everything except motor)
fuelcell.build_VoltageCurrentCurve()
fuelcell.plot_FCCurve()


## TRACK ##

#create track object
track = Track_c(SimulationTime,TimeInterval,TrackLength)
#set track parameters
track.Incline[1:500] = 0 #deg
track.Incline[500:750] = 5
track.Incline[750:1000] = -5
track.Incline[1000:1250] = 5
track.Incline[1250:1500] = -5
track.Incline[1500:1750] = 5
track.Incline[1750:2000] = -5
track.Incline[2000:2250] = 0
track.Incline[2500:2750] = 5
track.Incline[2750:3000] = -5
track.Incline[3000:3500] = 5
track.Incline[3500:4000] = -5
track.Incline[4000:4500] = 0
track.smoothtrack(5)
track.plot_Profile()
track.RelativeHumidity = 50 #%
track.Temperature = 30 #Celcius
track.AirPressure = 101 #kPa
track.calc_AirDensity()


## SUPERCAPS ##

#create super capacitor object
supercaps = SuperCapacitor_c(SimulationTime,TimeInterval)
#set super capacitor parameters
supercaps.Capacitance = 19.3

#### CAR ####

#create car object
car = Car_c(SimulationTime,TimeInterval)

## set car parameters ##
# NumberOfTeethDriven / NumberOfTeethDriving
car.GearRatio = 29 #unitless
# efficency of gears (based off friction etc)
car.GearEfficiency = 0.9 # spur gears usually over 90%
# Inertia of Gears. Not too important can be set to zero
car.GearInertia = 0
# total mass of everything
car.Mass = 245 #kg 
car.WheelDiameter = 0.56 # m
# I don't have an estimate for this yet. Its effect isn't very noticible anyway (I hope)
car.WheelInertia = 0 # kg m^2
# Constant rolling resistance ('Bearing Resistance')
car.BearingResistance = 10 #N #How do you estimate this?
#https://en.wikipedia.org/wiki/Rolling_resistance
car.RollingResistanceCoefficient = 0.01 #Unitless 
# http://physics.info/drag/
car.AreodynamicDragCoefficient = 0.3 # standard value for a car
car.FrontalArea = 1.2*1.67 #m^2

Simulation.run_Simulation(motor,fuelcell,car,track,supercaps,DataPoints,TimeInterval)


## Make Plots ##

motor.plot_TorqueSpeed()
motor.plot_PowerSpeed()
motor.plot_EfficiencySpeed()
motor.plot_SpeedTime()
motor.plot_TorqueTime()
motor.plot_CurrentTime()
motor.plot_VoltageTime()

fuelcell.plot_StackVoltageCurrent()
fuelcell.plot_StackEfficiency()
fuelcell.plot_StackCurrentTime()

car.plot_DistanceTime()
car.plot_SpeedTime()
car.plot_AccelerationTime()
car.plot_Milage()
car.plot_Drag()

supercaps.plot_VoltageCharge()
supercaps.plot_ChargeTime()
supercaps.plot_CurrentTime()

Simulation.plot_PowerCurves(fuelcell,motor,supercaps)