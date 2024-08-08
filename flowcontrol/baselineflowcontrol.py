import numpy as np
import networkx as nx
import osmnx as ox
from itertools import groupby
from copy import copy, deepcopy

import bluesky as bs
from bluesky import core, stack
from bluesky.tools.aero import kts
from bluesky.tools import datalog
from bluesky.tools import geo
from bluesky.traffic import Route
from bluesky.tools.misc import degto180


from plugins.utils import pluginutils


def init_plugin():

    bs.traf.flowcontrol = FlowControl()

    config = {
        'plugin_name'      : 'baselineflowcontrol',
        'plugin_type'      : 'sim',
        'reset'            : reset
        }

    return config

def reset():
    bs.traf.flowcontrol.reset()

class FlowControl(core.Entity):
    def __init__(self):
        super().__init__()

        self.geovector_time_limit = 60
        self.enableflowcontrol = False
        self.flowlog = datalog.crelog('FLOWLOG', None, flowheader)

        with self.settrafarrays():
            self.last_geovector = np.array([])


    def create(self, n=1):
        super().create(n)
        self.last_geovector[-n:] = -99999

    def reset(self):

        self.geovector_time_limit = 60
        self.enableflowcontrol = False
        self.flowlog = datalog.crelog('FLOWLOG', None, flowheader)

        # initialize the numpy random seed
        with self.settrafarrays():
            self.last_geovector = np.array([])

    @stack.command
    def ENABLEFLOWCONTROL(self):
        self.enableflowcontrol = True

    @stack.command
    def STARTFLOWLOG(self):
        self.flowlog.start()

    @stack.command
    def GEOTIME(self, geotime:int):
        self.geovector_time_limit = geotime

######################## FLOW CONTROL FUNCTIONS #########################

@core.timed_function(dt=bs.sim.simdt)
def do_flowcontrol():

    if bs.traf.ntraf == 0:
        return

    if not bs.traf.flowcontrol.enableflowcontrol:
        return
    
    # first apply some geovectors for aircraft
    apply_geovectors()

def apply_geovectors():

       # Give a speed limit depending on cluster
        bs.traf.selspd = np.where(bs.traf.swlnav, bs.traf.edgetraffic.actedge.speed_limit, bs.traf.selspd)


