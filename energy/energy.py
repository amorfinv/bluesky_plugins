""" 
BlueSky plugin that tracks how much battery life is left.
"""
import numpy as np
from bluesky.core import Entity, timed_function
from bluesky.sim import simt
from bluesky import stack, traf 

drone_range = None

def init_plugin():
    global drone_range
    drone_range = DroneRange()

    # Configuration parameters
    config = {
        'plugin_name'   :   'dronerange',
        'plugin_type'   :   'sim',
        'update'        :   update       
        }

    return config

def update() -> None:
    drone_range.update()



class DroneRange(Entity):
    ''' Example new entity object for BlueSky. '''
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.timerange = np.array([])
            self.timeleft = np.array([])
            self.spawntime = np.array([])

    def create(self, n=1):
        super().create(n)

        self.timerange[-n:] = 60
        self.timeleft[-n:] = 60
        self.spawntime[-n:] = simt



    def update(self):
        # every time step update the drone range

        # step 1:check how long drone has been in air
        time_in_air = simt - self.spawntime

        print(time_in_air)


