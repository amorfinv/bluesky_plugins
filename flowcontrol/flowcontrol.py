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

replanheader = \
    '#######################################################\n' + \
    'Cluster log LOG\n' + \
    'Cluster density statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'ACID [-], ' + \
    'Original plan [-], ' + \
    'Travelled plan [-], ' + \
    'issame [-], ' + \
    'n_replans [-], ' + \
    'replans [-]\n'



def init_plugin():

    bs.traf.flowcontrol = FlowControl()

    config = {
        'plugin_name'      : 'flowcontrol',
        'plugin_type'      : 'sim',
        'reset'            : reset
        }
    
    return config

def reset():
    bs.traf.flowcontrol.reset()

class FlowControl(core.Entity):
    def __init__(self):
        super().__init__() 

        self.replan_time_limit = 60
        self.enableflowcontrol = False
        self.flowlog = datalog.crelog('FLOWLOG', None, flowheader)
        self.replanlog = datalog.crelog('REPLANLOG', None, replanheader)

        # default replan ratio
        self.replan_ratio = 0.5

        # create the replan seeds
        self.replan_rng = np.random.default_rng()

        with self.settrafarrays():
            self.last_replan = np.array([])
            self.can_replan = np.array([])

  
    def create(self, n=1):
        super().create(n)
        self.last_replan[-n:] = -99999

    def reset(self):
        
        self.replan_time_limit = 60
        self.enableflowcontrol = False
        self.flowlog = datalog.crelog('FLOWLOG', None, flowheader)
        self.replanlog = datalog.crelog('REPLANLOG', None, replanheader)

        # default replan ratio
        self.replan_ratio = 0.5

        # create the replan seeds
        self.ac_replan_rng = np.random.default_rng()
        
        # initialize the numpy random seed
        with self.settrafarrays():
            self.last_replan = np.array([])
            self.can_replan = np.array([])

    @stack.command 
    def ENABLEFLOWCONTROL(self):
        self.enableflowcontrol = True
    
    @stack.command 
    def REPLANLIMIT(self, replan_time_limit:int):
        self.replan_time_limit = replan_time_limit

    @stack.command
    def REPLANRATIO(self, replanratio:float):
        # this sets the ratio of aircraft that can replan
        self.replan_ratio = replanratio

    @stack.command
    def FLOWSEED(self, flowseed:int):
        # this sets the ratio of aircraft that can replan
        self.ac_replan_rng = np.random.default_rng(flowseed)

    @stack.command 
    def STARTFLOWLOG(self):
        self.flowlog.start()

######################## FLOW CONTROL FUNCTIONS #########################

@core.timed_function(dt=10)
def do_flowcontrol():

    if not bs.traf.flowcontrol.enableflowcontrol:
        return
    
    # # start flow control at 10 mins
    # if bs.sim.simt <= 600:
    #     return

    # first apply some geovectors for aircraft
    apply_geovectors()

    # select which aircraft need to replan
    acid_to_replan = aircraft_to_replan()

    # replan the planns
    attempted_replans, succesful_replans, close_turn_replans, longer_replans_old, shorter_replans_old, shortest_path_replan = replan(acid_to_replan)

    # update logging
    update_logging(attempted_replans, succesful_replans, close_turn_replans, longer_replans_old, shorter_replans_old, shortest_path_replan)

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

        # perform the probability check
        # select number between 0 and 1
        replan_probability = bs.traf.flowcontrol.ac_replan_rng.random()

        # check if number is less than the replan ratio
        if not replan_probability <= bs.traf.flowcontrol.replan_ratio:
            continue
            
        # if aircraft has not done a replan recently then we get their unique travel edges
        unique_edges = bs.traf.TrafficSpawner.unique_edges[acidx]

        # now check if the unique edges are part of the edges that need a replan
        if not np.any(np.isin(unique_edges,bs.traf.clustering.cluster_edges)):
            continue

        # if it passes this step then we replan
        acid_to_replan.append((acid, acidx))

    return acid_to_replan

def replan(acid_to_replan):
    # start replanning

    # track total replans
    attempted_replans = len(acid_to_replan)
    succesful_replans = 0
    close_turn_replans = 0
    longer_replans_old = 0
    shorter_replans_old = 0
    shortest_path_replan = 0

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
        
        # NOTE: the current plan should not change from current edgeid to first entry of plan_edgeid which is the index_next_final
        
        # step 5: gather the data that will stay the same in the plan
        current_turn = bs.traf.edgetraffic.edgeap.edge_rou[acidx].turn[active_waypoint]
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
        
        # get old plan plan length
        old_nodes = deepcopy(bs.traf.TrafficSpawner.route_nodes[acidx])
        idx_old = old_nodes.index(int(node_start_plan))
        old_nodes = old_nodes[idx_old:]
        old_plan_length = get_route_length(old_nodes)

        # step 8: find a new route
        # TODO: merge it into one function with TrafficSpawner
        # TODO: check if plan actually changed
        # TODO: standardize edges as tuple
        # TODO: replan only affected area? 
        lats, lons, edges, turns, route_nodes = plan_path(int(node_start_plan),int(node_end_plan))
        
        # skip replan if a end of route
        if len(route_nodes) < 4:
            continue
        check_new_plan = [node not in bs.traf.TrafficSpawner.route_nodes[acidx] for node in route_nodes]
        # check if the plan has changed here
        if not np.any(check_new_plan):
            continue
        bs.traf.TrafficSpawner.route_nodes[acidx] = route_nodes

        # step 9: merge the starting routes with the new ones
        edges = start_edges + edges
        lats = np.concatenate((start_lats, lats))
        lons = np.concatenate((start_lons, lons))
        # create the turn bool with the updated information
        turns, _, _ = pluginutils.get_turn_arrays(lats, lons)

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

        # add this to the planned routes
        bs.traf.TrafficSpawner.planned_routes[acid].append(deepcopy(unique_edges))

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

        if turns[1] and not current_turn:

            # here log cases where the aircraft is forced to turn during a replan
            _,dist_between_points = geo.qdrdist(
                    bs.traf.lat[acidx], 
                    bs.traf.lon[acidx], 
                    lats[1], 
                    lons[1]
                    )
            dist_m = dist_between_points*bs.tools.aero.nm
            if dist_m < 5:
                close_turn_replans += 1

        # TODO: FINAL step is to gather information about the new plan and compare
        length_diff_true, length_diff_old = evaluate_new_route(route_nodes, node_start_plan, node_end_plan, old_plan_length)

        if length_diff_old > 0:
            longer_replans_old += 1
        else:
            shorter_replans_old += 1

        if length_diff_true > 0:
            shortest_path_replan += 1

    return attempted_replans, succesful_replans, close_turn_replans, longer_replans_old, shorter_replans_old, shortest_path_replan

def evaluate_new_route(route_nodes, node_start_plan, node_end_plan, old_plan_length):
    
    # get length of current plan
    length_new = get_route_length(route_nodes)
    
    # get true shortest path
    true_shortest_path = ox.shortest_path(bs.traf.TrafficSpawner.original_graph, int(node_start_plan), int(node_end_plan), weight='length')
    
    # get length of true shortest route
    length_true = get_route_length(true_shortest_path)
    # get difference of new length with true shortest path
    length_diff_true = length_new - length_true

    # get difference of new length with previous path
    length_diff_old = length_new - old_plan_length

    # now get a bunch of shortest paths with the real lengths
    #many_plans = ox.k_shortest_paths(bs.traf.TrafficSpawner.original_graph, int(node_start_plan), int(node_end_plan), 30, weight='length')
    # get length of first route
    #lengths_plans = []
    #idx_new_plan = None
    #for idx, plan in enumerate(many_plans):
    #    length = get_route_length(plan)
    #    lengths_plans.append(length)

    #    if plan == route_nodes:
    #        idx_new_plan = idx
    return length_diff_true, length_diff_old

def get_route_length(node_list):
    # get the true route length and not adjusted one
    edges_route = [(node_list[i], node_list[i+1],0) for i in range(len(node_list)-1)]
    length = bs.traf.TrafficSpawner.edges.loc[edges_route].length.sum()

    return length


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

def plan_path(orig_node, dest_node) -> None:
    
    # todo: CREM2 with nearest nodes
    node_route = ox.shortest_path(bs.traf.TrafficSpawner.graph, orig_node, dest_node, weight='length')
    
    # get lat and lon from route and turninfo
    lats, lons, edges = pluginutils.lat_lon_from_nx_route(bs.traf.TrafficSpawner.graph, node_route)
    # TODO: standardise with picklemaker
    turn_bool, _, _ = pluginutils.get_turn_arrays(lats, lons)
    
    return lats, lons, edges, turn_bool, node_route

