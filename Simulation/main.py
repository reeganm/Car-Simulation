# -*- coding: utf-8 -*-

from Motor_c import Motor_c
from FuelCell_c import FuelCell_c
from Track_c import Track_c
from Car_c import Car_c
from SuperCapacitor_c import SuperCapacitor_c

SimulationTime = 30 #30 seconds
TimeInterval = 0.0001 #0.0001 time step interval
TrackLength = 100 #100 meter track

#create motor object
motor = Motor_c(SimulationTime,TimeInterval)
#Set Motor Constants


#create fuelcell object
fuelcell = FuelCell_c(SimulationTime,TimeInterval)
#Set FuelCell parameters


#create track object
track = Track_c(SimulationTime,TimeInterval,TrackLength)
#set track parameters


#create super capacitor object
supercaps = SuperCapacitor_c(SimulationTime,TimeInterval)
#set super capacitor parameters


#create car object
car = Car_c(SimulationTime,TimeInterval)
#set car parameters

