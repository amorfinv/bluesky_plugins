import numpy as np
import geopandas as gpd
import pandas as pd
from shapely.geometry import Point
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

        self.geovector_update_time = 60
        self.geovector_active_time_limit = 30
        self.begin_geovector = 0
        self.flowlog = datalog.crelog('FLOWLOG', None, flowheader)

        with self.settrafarrays():
            self.last_geovector = np.array([])

  
    def create(self, n=1):
        super().create(n)
        self.last_geovector[-n:] = -99999

    def reset(self):

        self.geovector_update_time = 60
        self.geovector_active_time_limit = 30
        self.begin_geovector_time = 0
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
    def GEOTIME(self, geoactivetime:int, geotime:int):
        self.geovector_update_time = geotime
        self.geovector_active_time_limit = geoactivetime


######################## FLOW CONTROL FUNCTIONS #########################

@core.timed_function(dt=bs.sim.simdt)
def do_flowcontrol():

    if bs.traf.ntraf == 0:
        return
    
    if not bs.traf.flowcontrol.enableflowcontrol:
        return
    
    # set the classic speedlimit
    bs.traf.selspd = np.where(bs.traf.swlnav, bs.traf.edgetraffic.actedge.speed_limit, bs.traf.selspd)
    

    # only go into this code if past observation time since cluster polygons exist
    if bs.sim.simt > bs.traf.clustering.observation_time + bs.traf.flowcontrol.geovector_update_time:
        
        # now check if you are in a multiple of the
        if bs.sim.simt - bs.traf.flowcontrol.begin_geovector_time < bs.traf.flowcontrol.geovector_active_time_limit:
            # first apply some geovectors for aircraft
            apply_geovectors()
    
def apply_geovectors():
    
        # check if aircraft are intersecting a cluster polygon
        # First convert the aircraft positions to meters
        x,y = bs.traf.clustering.transformer_to_utm.transform(bs.traf.lat, bs.traf.lon)
        # define a geoseries
        point_list = [Point(xp,yp) for xp,yp in zip(x,y)]
        point_geoseries = gpd.GeoSeries(point_list, crs='EPSG:28992')
        bs.traf.clustering.aircraft_intersect(bs.traf.clustering.cluster_polygons, point_geoseries)

        flow_group_to_density = bs.traf.clustering.cluster_polygons.set_index('flow_group')['density_category'].to_dict()
        flow_groups_series = pd.Series(bs.traf.clustering.cluster_labels)
        density_series = flow_groups_series.map(lambda x: 1 if flow_group_to_density.get(x) == 'high' else 0)
        density_array = density_series.to_numpy()
        aircraft_to_slowdown = density_array
        speed_limits = np.where(aircraft_to_slowdown, 15*kts, 20*kts)

        bs.traf.selspd = np.where(bs.traf.swlnav, speed_limits, bs.traf.selspd)

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

