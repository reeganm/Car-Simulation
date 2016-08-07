# -*- coding: utf-8 -*-

import math

from Motor_c import Motor_c
from FuelCell_c import FuelCell_c
from Track_c import Track_c
from Car_c import Car_c
from SuperCapacitor_c import SuperCapacitor_c
from Simulation import run_Simulation

SimulationTime = 500 #seconds
TimeInterval = 0.001 #time step/integration interval #make a lot smaller than total inertia to decrease motor speed integration error
TrackLength = 38 # meters

DataPoints = math.floor(SimulationTime/TimeInterval)


#### MOTOR ####

#create motor object
motor = Motor_c(SimulationTime,TimeInterval)

## Set Motor Constants ##
#Velocity constant, Torque constant, BackEMFConstant are all related only need one https://en.wikipedia.org/wiki/Motor_constants
motor.VelocityConstant = motor.rmp_per_V_2_rad_per_Vs(113) # rad/Vs
motor.WindingResistance = 0.345 #ohms
# Manufacturers reported voltage
motor.MaxVoltage = 48 #Volts
# max speed and maxvoltage with no load
motor.MaxSpeed = motor.rmp_2_rad_per_s(5370) #rad/s
#inertia of motor armature (not too important can be set to zero)
motor.Inertia = motor.gcm2_2_kgm2(831) #kg m2
#set to limit motor current (ie some controllers limit current)  default is 1000 Amps
motor.MaxCurrent = 60 #Amp
#calculate other motor parameters
motor.calc_MissingMotorConstants()


## FUELCELL ##

#create fuelcell object
fuelcell = FuelCell_c(SimulationTime,TimeInterval)
#Set FuelCell parameters
fuelcell.CellNumber = 46
fuelcell.CellArea = 250
fuelcell.CellResistance = 0.62
fuelcell.Alpha = 0.6
fuelcell.ExchangeCurrentDensity = 0.04
fuelcell.CellOCVoltage = 0.956 #open circuit voltage
fuelcell.DiodeVoltageDrop = 0.7
fuelcell.AuxCurrent = 2 #current consumed by controllers, fans etc (everything except motor)


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


#### CAR ####

#create car object
car = Car_c(SimulationTime,TimeInterval)

## set car parameters ##
# NumberOfTeethDriven / NumberOfTeethDriving
car.GearRatio = 8.0 #unitless
# efficency of gears (based off friction etc)
car.GearEfficiency = 0.9 # spur gears usually over 90%
# Inertia of Gears. Not too important can be set to zero
car.GearInertia = 0.05
# total mass of everything
car.Mass = 310 #kg 
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

run_Simulation(motor,fuelcell,car,track,supercaps,DataPoints,TimeInterval)


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

car.plot_DistanceTime()
car.plot_SpeedTime()