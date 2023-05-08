""" 
BlueSky plugin that tracks how much battery life is left.
"""
import numpy as np
import pandas as pd

import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky import sim
from bluesky import stack, traf 

drone_energy = None

def init_plugin():
    global drone_energy
    drone_energy = DroneEnergy()

    # Configuration parameters
    config = {
        'plugin_name'   :   'energy',
        'plugin_type'   :   'sim',
        'update'        :   update       
        }

    return config



class DroneEnergy(Entity):
    ''' Example new entity object for BlueSky. '''
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            
            self.payloadmass = np.array([])
            self.batterymass = np.array([])
            self.dronemass = np.array([])
            
            self.droneCd = np.array([])
            self.batteryCd = np.array([])
            self.payloadCd = np.array([])

            self.dronearea = np.array([])
            self.batteryarea = np.array([])
            self.payloadarea = np.array([])

            self.nrotors = np.array([])
            self.discarea = np.array([])
            self.rotordiameter = np.array([])

            self.battery_capacity = np.array([])
            self.remaining_capacity = np.array([])
            self.energy_threshold = np.array([])

            self.timerange = np.array([])
            self.timeleft = np.array([])
            self.spawntime = np.array([])

        # load the csv of the energy budgets
        self.energy_db = pd.read_csv('plugins/energy/energy.csv')

    def create(self, n=1):
        super().create(n)

        self.timerange[-n:] = 60
        self.timeleft[-n:] = 60
        self.spawntime[-n:] = sim.simt
        
        self.dronemass[-n:] = 7
        self.batterymass[-n:] = 10
        self.payloadmass[-n:] = 7

        self.droneCd[-n:] = 1.49
        self.batteryCd[-n:] = 1
        self.payloadCd[-n:] = 2.2

        self.dronearea[-n:] = 0.224
        self.batteryarea[-n:] = 0.015
        self.payloadarea[-n:] = 0.0929

        self.battery_capacity[-n:] = (540000*10)/1.2 # specific energy capacity 540,000 J/kg times mass/ safety factor
        self.remaining_capacity[-n:] = (540000*10)/1.2
        self.energy_threshold[-n:] = 0.1*(540000*10)/1.2

        self.nrotors[-n:] = 8
        self.discarea[-n:] = 0.027
        self.rotordiameter[-n:] = 0.432

def update():
    # every time step check if remaining energy is less than threshold
    drones_to_land = np.where(drone_energy.remaining_capacity < drone_energy.energy_threshold)

    # get drones
    id_array = np.array(bs.traf.id)
    
    # select those with 10 percent
    drones_to_land = id_array[drones_to_land]

    print(drones_to_land)

@timed_function(dt=1)
def energy_spend():
    
    # reduce energy budget of all aircraft
    # check current speed of aircraft

    for idx, acid in enumerate(bs.traf.id):

        airspeed = round(bs.traf.tas[idx])
        
        payload_mass = drone_energy.payloadmass[idx]
        battery_mass = drone_energy.batterymass[idx]
        drone_mass = drone_energy.dronemass[idx]
        
        payload_choice = 'power_payload' if payload_mass > 0 else 'power_no_payload'

        # get expended power
        power_expended = drone_energy.energy_db[drone_energy.energy_db['airspeed']==airspeed][payload_choice].values[0]

        # remove from energy budget
        drone_energy.remaining_capacity[idx] -= power_expended*1  # power*dt

        # estimate time remaining

        # estimate range remaining


