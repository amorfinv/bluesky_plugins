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
            
            self.spawn_time = np.array([])
            
            self.payload_mass = np.array([])
            self.battery_mass = np.array([])
            self.drone_mass = np.array([])

            self.battery_capacity = np.array([])
            self.remaining_capacity = np.array([])
            self.energy_threshold = np.array([])

            self.range_remaining = np.array([])
            self.time_remaining = np.array([])


        # load the csv of the energy budgets
        self.energy_db = pd.read_csv('plugins/energy/energy.csv')

    def create(self, n=1):
        super().create(n)

        specific_energy = 540000 # specific energy capacity 540,000 J/kg
        battery_mass = 10 # mass of battery kg
        drone_mass = 7
        payload_mass = 7 
        safety_factor = 1.2
        energy_threshold = 0.1 # 10 percent
        
        self.spawn_time[-n:] = sim.simt
        
        self.drone_mass[-n:] = drone_mass
        self.battery_mass[-n:] = battery_mass
        self.payload_mass[-n:] = payload_mass

        self.battery_capacity[-n:] = (specific_energy*battery_mass)/safety_factor # specific energy capacity x mass / safety factor
        self.remaining_capacity[-n:] = (specific_energy*battery_mass)/safety_factor
        self.energy_threshold[-n:] = energy_threshold*(specific_energy*battery_mass)/safety_factor

        self.range_remaining[-n:] = 15000
        self.time_remaining[-n:] = 15000

def update():
    # every time step check if remaining energy is less than threshold
    drones_below_threshold = np.where(drone_energy.remaining_capacity < drone_energy.energy_threshold)

    # get drones
    id_array = np.array(bs.traf.id)
    
    # select those with 10 percent
    drones_to_land = id_array[drones_below_threshold]
    
    # if len(drones_to_land):
    #     print(drone_energy.range_remaining[drones_below_threshold])
    #     print(drones_to_land)

@timed_function(dt=1)
def energy_spend():
    
    # reduce energy budget of all aircraft
    # check current speed of aircraft

    for idx, acid in enumerate(bs.traf.id):

        airspeed = round(bs.traf.tas[idx])
        
        payload_mass = drone_energy.payload_mass[idx]
        battery_mass = drone_energy.battery_mass[idx]
        drone_mass = drone_energy.drone_mass[idx]
        
        payload_choice = 'power_payload' if payload_mass > 0 else 'power_no_payload'

        # get expended power
        power_expended = drone_energy.energy_db[drone_energy.energy_db['airspeed']==airspeed][payload_choice].values[0]

        # remove from energy budget
        drone_energy.remaining_capacity[idx] -= power_expended*1  # power*dt

        # estimate range remaining
        drone_energy.range_remaining[idx] = (0.25/1.2)*(drone_energy.remaining_capacity[idx]*airspeed)/power_expended

        # estimated time remaining
        drone_energy.time_remaining[idx] = drone_energy.range_remaining[idx]/airspeed


