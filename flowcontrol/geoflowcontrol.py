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


flowheader = \
    '#######################################################\n' + \
    'Cluster log LOG\n' + \
    'Cluster density statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Simulation time [s], ' + \
    'Number of aircraft in air [-], ' + \
    'Number of attempted replans[-], ' + \
    'Number of successful replans[-], ' + \
    'Succesful replans/attempted replans * 100[%], ' + \
    'Succesful replans / Number of aircraft in air * 100 [%], ' + \
    'Close turn replans [-]' + \
    'Number of replans resulting in longer distance [-]' + \
    'Number of replan resulting in shorter distance [-]' + \
    'Number of replans giving shortest path [-]\n'



def init_plugin():

    bs.traf.flowcontrol = FlowControl()

    config = {
        'plugin_name'      : 'geoflowcontrol',
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
    
    # # only run on multiples of geovector time limit
    # if bs.sim.simt % bs.traf.flowcontrol.geovector_time_limit:
    #     return
    
    # first apply some geovectors for aircraft
    apply_geovectors()

    # # update logging
    # update_logging(attempted_replans, succesful_replans, close_turn_replans, longer_replans_old, shorter_replans_old, shortest_path_replan)

def apply_geovectors():
        
        in_turn = np.logical_or(bs.traf.ap.inturn, bs.traf.ap.dist2turn < 75)
        cr_active = bs.traf.cd.inconf
        lnav_on = bs.traf.swlnav

        # if bs.sim.simt > 600:
        #     print(bs.traf.id[0])
        #     print(bs.traf.edgetraffic.actedge.speed_limit[0])

       # Give a speed limit depending on cluster
        bs.traf.selspd = np.where(bs.traf.swlnav, bs.traf.edgetraffic.actedge.speed_limit, bs.traf.selspd)
        # if bs.sim.simt > 600:
        #     print(bs.traf.selspd[0])
        #     print('--------------')

        # bs.traf.selspd = np.where(bs.traf.swlnav, bs.traf.actedge.speed_limit, bs.traf.selspd)
    

def update_logging(attempted_replans, succesful_replans, close_turn_replans, longer_replans_old, shorter_replans_old, shortest_path_replan):
    if attempted_replans == 0:
        return

    bs.traf.flowcontrol.flowlog.log(
        bs.traf.ntraf,
        attempted_replans,
        succesful_replans,
        (succesful_replans/attempted_replans) * 100,
        (succesful_replans/bs.traf.ntraf) * 100,
        close_turn_replans,
        longer_replans_old,
        shorter_replans_old,
        shortest_path_replan
    )

    
    # todo: CREM2 with nearest nodes
    node_route = ox.shortest_path(bs.traf.TrafficSpawner.graph, orig_node, dest_node, weight='length')
    
    # get lat and lon from route and turninfo
    lats, lons, edges = pluginutils.lat_lon_from_nx_route(bs.traf.TrafficSpawner.graph, node_route)
    # TODO: standardise with picklemaker
    turn_bool, _, _ = pluginutils.get_turn_arrays(lats, lons)
    
    return lats, lons, edges, turn_bool, node_route

