import json
import numpy as np
from collections import Counter
import pandas as pd
from scipy.sparse import csr_matrix
from scipy.spatial import KDTree
import networkx as nx
import osmnx as ox
from itertools import groupby

import bluesky as bs
from bluesky import core, stack, traf, scr, sim
from bluesky.tools.aero import ft, kts, nm
from bluesky.tools import geo
from bluesky.core import Entity, Replaceable
from bluesky.traffic import Route
from bluesky.tools.misc import degto180, txt2tim, txt2alt, txt2spd, lat2txt

from plugins.utils import pluginutils

# # create classes
flowcontrol = None

def init_plugin():

    bs.traf.flowcontrol = FlowControl()

    config = {
        'plugin_name'      : 'flowcontrol',
        'plugin_type'      : 'sim',
        'reset'            : bs.traf.flowcontrol.reset()
        }
    
    return config
class FlowControl(core.Entity):
    def __init__(self):
        super().__init__() 

        self.replan_time_limit = 60
        self.enableflowcontrol = False

        with self.settrafarrays():
            self.last_replan = np.array([])
  
    def create(self, n=1):
        super().create(n)
        self.last_replan[-n:] = bs.sim.simt

    @stack.command 
    def ENABLEFLOWCONTROL(self):
        self.enableflowcontrol = True

    def reset(self):
        self.enableflowcontrol = False

######################## FLOW CONTROL FUNCTIONS #########################

@core.timed_function(dt=10)
def do_flowcontrol():

    if not bs.traf.flowcontrol.enableflowcontrol:
        return
        
    # first apply some geovectors for aircraft
    apply_geovectors()

    # select which aircraft need to replan
    acid_to_replan = aircraft_to_replan()

    # replan the planns
    replan(acid_to_replan)

def apply_geovectors():
    pass


def aircraft_to_replan():
    
    # TODO: choose stroke groups to check replan
    # TODO: select only from current edge to end
    # select which aircraft should replan

    acid_to_replan = []

    for acidx, acid in enumerate(bs.traf.id):

        # First check that the current aircraft has not done a replan 
        # within the self.replan time limit
        replan_time = bs.sim.simt - bs.traf.flowcontrol.last_replan[acidx]
        if replan_time <  bs.traf.flowcontrol.replan_time_limit:
            continue
            
        # if aircraft has not done a replan then we get their unique travel edges
        unique_edges = bs.traf.TrafficSpawner.unique_edges[acidx]

        # now check if the unique edges are part of the edges that need a replan
        if np.any(np.isin(unique_edges,bs.traf.clustering.cluster_edges)):
            acid_to_replan.append((acid, acidx))
    
    return acid_to_replan


def replan(acid_to_replan):
    # start replanning

    for acid, acidx in acid_to_replan:

        # step 1 get current edge of travel
        current_edgeid = bs.traf.edgetraffic.actedge.wpedgeid[acidx]

        # step 2 get activewaypoint
        active_waypoint = bs.traf.edgetraffic.edgeap.edge_rou[acidx].iactwp

        # step 3, start plan at the next edge
        # TODO: move this to streets autopilot update? or just use active_waypoint
        index_unique = np.argwhere(bs.traf.TrafficSpawner.unique_edges[acidx] == current_edgeid)[0][0]
        
        # no replan if near end of route 
        # TODO: make it smarter
        if len(bs.traf.TrafficSpawner.unique_edges[acidx][index_unique:]) < 4:
            return

        plan_edgeid = bs.traf.TrafficSpawner.unique_edges[acidx][index_unique+1]
        index_start_plan = bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid.index(plan_edgeid)

        # NOTE: the current plan should not change from current edgeid to first entry of plan_edgeid which is the index_next_final
        
        # step 5: gather the data that will stay the same in the plan
        start_edges = bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid[active_waypoint:index_start_plan]
        start_turns = np.array(bs.traf.edgetraffic.edgeap.edge_rou[acidx].turn[active_waypoint:index_start_plan])
        start_lats  = np.array(bs.traf.ap.route[acidx].wplat[active_waypoint:index_start_plan])
        start_lons  = np.array(bs.traf.ap.route[acidx].wplon[active_waypoint:index_start_plan])

        # start_edges = [start_edges[0]] + start_edges
        start_turns = np.insert(start_turns, 0, bs.traf.edgetraffic.edgeap.edge_rou[acidx].turn[active_waypoint])
        start_lats = np.insert(start_lats, 0, bs.traf.lat[acidx])
        start_lons = np.insert(start_lons, 0, bs.traf.lon[acidx])

        # step 6: now able to delete current route
        # If the next waypoint is a turn waypoint, then remember the turnrad
        nextqdr_to_remember = bs.traf.actwp.next_qdr[acidx]
        acrte = Route._routes.get(acid)
        bs.traf.edgetraffic.edgeap.update_route(acid, acidx)

        # step 7: select node to start plan from and final node
        node_start_plan = plan_edgeid.split('-')[0]
        node_end_plan = bs.traf.TrafficSpawner.unique_edges[acidx][-1].split('-')[1]

        # step 8: find a new route
        # TODO: merge it into one function with TrafficSpawner
        # TODO: check if plan actually changed
        # TODO: standardize edges as tuple
        # TODO: replan only affected area? 
        lats, lons, edges, turns = plan_path(int(node_start_plan),int(node_end_plan))

        # step 9: merge the starting routes with the new ones
        edges = start_edges + edges
        lats = np.concatenate((start_lats, lats))
        lons = np.concatenate((start_lons, lons))
        turns = np.concatenate((start_turns, turns))

        bs.traf.TrafficSpawner.route_edges[acidx] = edges
        unique_edges = list({edge: None for edge in edges}.keys())
        bs.traf.TrafficSpawner.unique_edges[acidx] = np.array(unique_edges)

        # Start adding waypoints
        for edgeidx, lat, lon, turn in zip(edges, lats, lons, turns):
            if turn:
                acrte.turnspd = 5 * kts
                acrte.swflyby = False
                acrte.swflyturn = True
            else:
                acrte.swflyby = True
                acrte.swflyturn = False
                
            wptype  = Route.wplatlon
            acrte.addwpt_simple(acidx, acid, wptype, lat, lon, bs.traf.TrafficSpawner.alt, bs.traf.TrafficSpawner.spd)

            bs.traf.edgetraffic.edgeap.edge_rou[acidx].addwpt(acidx, acid, edgeidx, turn)
        
        # Calculate the flight plan
        acrte.calcfp()
        bs.traf.edgetraffic.edgeap.edge_rou[acidx].direct(acidx,bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpname[1])
        stack.stack(f'LNAV {acid} ON')
        stack.stack(f'VNAV {acid} ON')
            
        # # Add the turndist back
        if not bs.traf.actwp.flyby[acidx] and bs.traf.actwp.flyturn[acidx]:
            bs.traf.actwp.next_qdr[acidx] = nextqdr_to_remember

        # update the last replan time
        bs.traf.flowcontrol.last_replan[acidx] = bs.sim.simt

        # check if route has a self loop SANITY CHECK
        route_edges = bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid

        unique_continuous_edges = [key for key, _group in groupby(route_edges)]
        set_continuous_edges = set(unique_continuous_edges)
        
        if len(unique_continuous_edges) != len(set_continuous_edges):
            print('ERROR NON continuous edges!')

def plan_path(orig_node, dest_node) -> None:
    
    # todo: CREM2 with nearest nodes
    node_route = nx.shortest_path(bs.traf.TrafficSpawner.graph, orig_node, dest_node, weight='adjusted_length', method='dijkstra')

    # get lat and lon from route and turninfo
    lats, lons, edges, _ = pluginutils.lat_lon_from_nx_route(bs.traf.TrafficSpawner.graph, node_route)

    # TODO: standardise with picklemaker
    turn_bool, turn_speed, turn_coords = pluginutils.get_turn_arrays(lats, lons)

    # get initial bearing
    qdr, _ = geo.qdrdist(lats[0], lons[0], lats[1], lons[1])
    
    return lats, lons, edges, turn_bool

