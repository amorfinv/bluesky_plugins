import numpy as np

import bluesky as bs
from bluesky.core import Entity
from bluesky.tools.aero import rho0, g0


# initialize drone stats
dronebattery = None

def init_plugin():
    global dronebattery
    dronebattery = DroneBattery()

    config = {
        'plugin_name': 'dronebattery',
        'plugin_type': 'sim',
    }
    return config


class DroneBattery(Entity):

    def __init__(self):
        super().__init__()

        with self.settrafarrays():

            # Battery information
            self.batt_senergy           = np.array([])              # specific energy of the battery (J/kg)
            self.batt_eff               = np.array([])              # battery power transfer efficiency
            self.batt_sfactor           = np.array([])              # safety factor of battery
            self.depth_discharge        = np.array([])              # depth of discharge of battery
            self.batt_mass              = np.array([])              # mass of the battery (kg)
            self.batt_area              = np.array([])              # projected area of the battery (m^2)
            self.batt_Cd                = np.array([])              # drag coefficient of the battery

            # Rotor information
            self.blades_per_rotor       = np.array([])              # number of blades in one rotor
            self.blade_chord_length     = np.array([])              # length of blade chord (m)
            self.blade_Cl               = np.array([])              # blade lift coefficient
            self.blade_Cd               = np.array([])              # blade drag coefficient
            self.n_rotors               = np.array([])              # number of rotors
            self.disc_area              = np.array([])              # disc area (m^2)

            # drone information
            self.drone_mass             = np.array([])              # mass of drone body (kg)
            self.drone_area             = np.array([])              # projected area of the drone body (m^2)
            self.drone_Cd               = np.array([])              # drag coefficient of drone
            
            # payload info
            self.payload_mass           = np.array([])              # mass of payload (kg)
            self.payload_area           = np.array([])              # projected area of payload (m^2)
            self.payload_Cd             = np.array([])              # drag coefficient of the payload

            # general
            self.lift_to_drag           = np.array([])              # lift to drag ratio
            self.power_avionics         = np.array([])              # power required for avionics (J/s)
            self.induced_power_f        = np.array([])              # factor for induced power
            self.profile_power_f        = np.array([])              # factor for profile power (m/kg)^(1/2)
            self.rofile_power_v_f       = np.array([])              # factor for profile power due to speed (m/kg)^(-1/2)
            self.parasite_power_pay_f   = np.array([])              # factor for parasite power with payload (kg/m)
            self.parasite_power_f       = np.array([])              # factor for parasite power without payload (kg/m)
            
    
    def create(self, n=1):
        super().create(n)

        # Battery information
        self.batt_senergy[-n:]           = 540000       
        self.batt_eff[-n:]               = 0.7
        self.batt_sfactor[-n:]           = 1.2
        self.depth_discharge[-n:]        = 0.5
        self.batt_mass[-n:]              = 10
        self.batt_area[-n:]              = 0.015
        self.batt_Cd[-n:]                = 1

        # Rotor information
        self.blades_per_rotor[-n:]       = 3
        self.blade_chord_length[-n:]     = 0.1
        self.blade_Cl[-n:]               = 0.4
        self.blade_Cd[-n:]               = 0.075
        self.n_rotors[-n:]               = 8
        self.disc_area[-n:]              = 0.027

        # drone information
        self.drone_mass[-n:]             = 7
        self.drone_area[-n:]             = 0.224
        self.drone_Cd[-n:]               = 1.49
        
        # payload info
        self.payload_mass[-n:]           = 7
        self.payload_area[-n:]           = 0.0929
        self.payload_Cd[-n:]             = 2.2

        # general
        self.lift_to_drag[-n:]           = 3
        self.power_avionics[-n:]         = 0
        self.induced_power_f[-n:]        = 1
        self.profile_power_f[-n:]        = 0.683
        self.rofile_power_v_f[-n:]       = 0.0868
        self.parasite_power_pay_f[-n:]   = 0.339
        self.parasite_power_f[-n:]       = 0.214
    

    def stolaroff_model(self):

        # first step is to calculate angle of attack (airspeed, drone rotor) (EQ.15)
        force_drag = 0.5*rho0*(self.payload_area*self.payload_Cd + self.batt_area*self.batt_Cd + self.drone_area*self.drone_Cd)*np.square(bs.traf.tas)
        force_grav = g0*(self.payload_mass + self.batt_mass + self.drone_mass)
        angle_of_attack = np.atan((force_drag)/(force_grav))

        # second step is to calculate the induced speed through disc area
        trig_term = bs.traf.tas * np.cos(angle_of_attack) + np.square(bs.traf.tas*np.sin(angle_of_attack) + vi)
        vi = force_grav / (2*self.n_rotors*rho0*self.disc_area)


    def update(): ...

'''

TODO: decide for vienna or orthogonal city?

PLugin format

Plugin A:

    - create traffic patterns based on a point distribution
    - in first implementation the origin/destination points are attempted to be uniformly distributed as much as possible
    - not always possible due to the city arrangement?
    - perhaps work on 
    - in future create different patterns
    - types of missions
        1) spawn: constrained airspace. delete: open airspace
        2) spawn: open airspace. delete: constrained airspace
        3) spawn: constrained airspace. delete: constrained airsapce
        4) spawn: open airspace. delete: open airspace
    - all missions should go through constrained airspace so remove them
    - maintain a given number of aircraft in the air
    - spawn aircraft above their origin at cruise speed
    - delete aircraft above their destination. 

Plugin B:

    - This plugin should aggregate geo-statistcs every so often
    - statistics should conflicts/intrusions/density
    - conflicts intrusions should have 'memory' perhaps statistics are saved for the last 10 minutes or so

Plugin C

    - identify clusters based on data
    - use k-means or other strategies

Plugin D

    - modify the weight of the graph given the live cluster analysis. this plugin should be active every 10 seconds or so

Plugin E

    - given the new graph weights identify which aircraft should attempt a replan and then replan


Considerations: use cuda graphs instead of networkx if you can get ahold of a gpu?


Future:
    - add wind for weights of the graph
    - add battery consumption for graph
    - consider non-uniform point distribution..converging or diverging traffic flows
    - use ML methods for clustering
    - use ML methods for graph weight setting

'''
# what should plugin do
# 1 spawn aircraft
# 2 types of spawning