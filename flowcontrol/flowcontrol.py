import json
import numpy as np
from collections import Counter
import pandas as pd
from scipy.sparse import csr_matrix
from scipy.spatial import KDTree
import networkx as nx
import osmnx as ox

import bluesky as bs
from bluesky import core, stack, traf, scr, sim
from bluesky.tools.aero import ft, kts, nm
from bluesky.tools import geo
from bluesky.core import Entity, Replaceable
from bluesky.traffic import Route
from bluesky.tools.misc import degto180, txt2tim, txt2alt, txt2spd, lat2txt

from plugins.utils import pluginutils

# # create classes
# edge_traffic = None
# path_plans = None

def init_plugin():
    
    global path_plans, edge_traffic

    # # Initialize EdgeTraffic
    # edge_traffic = EdgeTraffic(bs.settings.graph_dir)

    # # Initialize Path Plans
    # path_plans = PathPlans(bs.settings.graph_dir)

    config = {
        'plugin_name'      : 'flowcontrol',
        'plugin_type'      : 'sim',
        # 'update':          update,
        # 'reset':           reset
        }
    
    return config

######################## FLOW CONTROL FUNCTIONS #########################

@core.timed_function(dt=10)
def do_flowcontrol():

    # select which aircraft need to replan
    acid_to_replan = aircraft_to_replan()

    replan(acid_to_replan)

    # apply new weights to graph

    # replan aircraft that need to replan

def aircraft_to_replan():
    
    # TODO: choose stroke groups to check replan
    # TODO: select only from current edge to end
    # select which aircraft should replan

    acid_to_replan = []

    for acidx, acid in enumerate(bs.traf.id):
        unique_edges = bs.traf.TrafficSpawner.unique_edges[acidx]

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

        # step 3, get the id of the next edge id
        # TODO: move this to streets autopilot update? or just use active_waypoint
        index_current = np.argwhere(bs.traf.TrafficSpawner.unique_edges[acidx] == current_edgeid)[0][0]
        next_edgeid = bs.traf.TrafficSpawner.unique_edges[acidx][index_current+1]

        # step 4: find the last of the next_edgeid in route
        index_next_final = len(bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid) - bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid[::-1].index(next_edgeid) - 1

        # NOTE: the current plan should not change from current edgeid to last entry of the next edgeid which is the index_next_final
        
        # step 5: gather the data that will stay the same in the plan
        start_edges = bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid[:index_next_final+1]
        start_turns = np.array(bs.traf.edgetraffic.edgeap.edge_rou[acidx].turn[:index_next_final+1])
        start_lats = bs.traf.lat[:index_next_final+1]
        start_lons = bs.traf.lon[:index_next_final+1]

        # step 6: now able to delete current route
        acrte = Route._routes.get(acid)
        bs.traf.edgetraffic.edgeap.update_route(acid, acidx)

        # step 7: select node to start plan from and final node
        node_start_plan = next_edgeid.split('-')[1]
        node_end_plan = bs.traf.TrafficSpawner.unique_edges[acidx][-1].split('-')[1]

        # step 8: find a new route
        # TODO: merge it into one function with TrafficSpawner
        # TODO: standardize edges as tuple
        # TODO: replan only affected area? 
        lats, lons, edges, turns = plan_path(int(node_start_plan),int(node_end_plan))
        bs.traf.TrafficSpawner.route_edges[acidx] = edges
        unique_edges = list({edge: None for edge in edges}.keys())
        bs.traf.TrafficSpawner.unique_edges[acidx] = np.array(unique_edges)

        # step 9: merge the starting routes with the new ones
        edges = start_edges + edges
        lats = np.concatenate((start_lats, lats))
        lons = np.concatenate((start_lons, lons))
        turns = np.concatenate((start_turns, turns))

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

