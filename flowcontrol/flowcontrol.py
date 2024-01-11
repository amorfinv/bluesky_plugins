import numpy as np
import networkx as nx
from itertools import groupby
from copy import copy
import random

import bluesky as bs
from bluesky import core, stack
from bluesky.tools.aero import kts
from bluesky.tools import datalog
from bluesky.tools import geo
from bluesky.traffic import Route

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
    'Succesful replans / Number of aircraft in air * 100 [%]\n'


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
        self.flowlog = datalog.crelog('FLOWLOG', None, flowheader)
        
        # default replan ratio
        self.replan_ratio = 0.5
        self.replan_probabilities = [self.replan_ratio, 1-self.replan_ratio]


        with self.settrafarrays():
            self.last_replan = np.array([])
            self.can_replan = np.array([])

  
    def create(self, n=1):
        super().create(n)
        self.last_replan[-n:] = bs.sim.simt
        self.can_replan[-n:] = random.choices([True, False], self.replan_probabilities)


    @stack.command 
    def ENABLEFLOWCONTROL(self):
        self.enableflowcontrol = True
    
    @stack.command 
    def REPLANLIMIT(self, replan_time_limit:int):
        self.replan_time_limit = replan_time_limit

    @stack.command
    def REPLANRATIO(self, replanratio:float):
        # this sets the ratio of aircraft that can replan
        # number between 0 and 1.
        self.replan_ratio = replanratio
        self.replan_probabilities = [self.replan_ratio, 1-self.replan_ratio]

    @stack.command 
    def STARTFLOWLOG(self):
        self.flowlog.start()

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
    attempted_replans, succesful_replans = replan(acid_to_replan)

    # update logging
    update_logging(attempted_replans, succesful_replans)

def apply_geovectors():
    pass


def aircraft_to_replan():
    
    # TODO: choose stroke groups to check replan
    # TODO: select only from current edge to end
    # select which aircraft should replan

    acid_to_replan = []

    for acidx, acid in enumerate(bs.traf.id):

        if not bs.traf.flowcontrol.can_replan[acidx]:
            continue
        
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

    # track total replans
    attempted_replans = len(acid_to_replan)
    succesful_replans = 0

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
            continue

        plan_edgeid = bs.traf.TrafficSpawner.unique_edges[acidx][index_unique+1]
        index_start_plan = bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid.index(plan_edgeid)

        # also save old plan to see if it has changed
        old_edgeids = copy(bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid)[active_waypoint:]
        
        # NOTE: the current plan should not change from current edgeid to first entry of plan_edgeid which is the index_next_final
        
        # step 5: gather the data that will stay the same in the plan
        start_edges = bs.traf.edgetraffic.edgeap.edge_rou[acidx].wpedgeid[active_waypoint:index_start_plan]
        start_turns = np.array(bs.traf.edgetraffic.edgeap.edge_rou[acidx].turn[active_waypoint:index_start_plan])
        start_lats  = np.array(bs.traf.ap.route[acidx].wplat[active_waypoint:index_start_plan])
        start_lons  = np.array(bs.traf.ap.route[acidx].wplon[active_waypoint:index_start_plan])

        # add the current position of the aircraft to the front
        start_edges = [copy(start_edges[0])] + copy(start_edges)
        start_turns = np.insert(start_turns, 0, bs.traf.edgetraffic.edgeap.edge_rou[acidx].turn[active_waypoint])
        start_lats = np.insert(start_lats, 0, bs.traf.lat[acidx])
        start_lons = np.insert(start_lons, 0, bs.traf.lon[acidx])

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

        # here check if plan has changed, check from second entry in new edges 
        # because the first entry is the current aircraft position
        new_edgeids = copy(edges[1:])
        if old_edgeids == new_edgeids:
            # if entering here then we have the same plan
            # do not replan
            continue

        # increment succesful replans
        succesful_replans += 1

        # now able to delete current route
        # If the next waypoint is a turn waypoint, then remember the turnrad
        nextqdr_to_remember = bs.traf.actwp.next_qdr[acidx]
        acrte = Route._routes.get(acid)
        bs.traf.edgetraffic.edgeap.update_route(acid, acidx)

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

    # print(attempted_replans, succesful_replans)
    return attempted_replans, succesful_replans

def update_logging(attempted_replans, succesful_replans):

    if attempted_replans == 0:
        return

    bs.traf.flowcontrol.flowlog.log(
        bs.traf.ntraf,
        attempted_replans,
        succesful_replans,
        (succesful_replans/attempted_replans) * 100,
        (succesful_replans/bs.traf.ntraf) * 100
    )

def plan_path(orig_node, dest_node) -> None:
    
    # todo: CREM2 with nearest nodes
    node_route = nx.shortest_path(bs.traf.TrafficSpawner.graph, orig_node, dest_node, weight='adjusted_length', method='dijkstra')

    # get lat and lon from route and turninfo
    lats, lons, edges, _ = pluginutils.lat_lon_from_nx_route(bs.traf.TrafficSpawner.graph, node_route)

    # TODO: standardise with picklemaker
    turn_bool, _, _ = pluginutils.get_turn_arrays(lats, lons)

    # get initial bearing
    _, _ = geo.qdrdist(lats[0], lons[0], lats[1], lons[1])
    
    return lats, lons, edges, turn_bool

