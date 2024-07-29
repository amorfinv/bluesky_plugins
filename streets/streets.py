"""
Airspace edge information. Emulates traffic, autopilot, route, activewaypoint.
Also includes layering information.
"""

import json
import numpy as np
import pandas as pd
import osmnx as ox
from scipy.sparse import csr_matrix
from scipy.spatial import KDTree
import networkx as nx

import bluesky as bs
from bluesky import core, stack, traf
from bluesky.tools.aero import ft, kts, nm
from bluesky.tools import geo
from bluesky.core import Entity, Replaceable
from bluesky.traffic import Route

from plugins.utils import pluginutils

bs.settings.set_variable_defaults(
    graph_dir=f'plugins/scenario_maker/Rotterdam/')

# create classes
edge_traffic = None
path_plans = None

def init_plugin():
    
    global path_plans, edge_traffic

    # Initialize EdgeTraffic
    edge_traffic = EdgeTraffic(bs.settings.graph_dir)

    # Initialize Path Plans
    path_plans = PathPlans(bs.settings.graph_dir)

    config = {
        'plugin_name'      : 'streets',
        'plugin_type'      : 'sim',
        'update':          update,
        'reset':           reset
        }
    
    return config

######################## UPDATE FUNCTION  ##########################

def update():
    global streets_bool

    if streets_bool:
        # Main update function for streets plugin. updates edge and flight layer tracking
        
        # Update edege autopilot
        edge_traffic.edgeap.update()

####################### RESET FUNCTION ###############################

def reset():
    # when reseting bluesky turn off streets
    global streets_bool
    streets_bool = False
    
    bs.traf.ap.idxreached = []
######################## STACK COMMANDS ##########################

@stack.command
def CREM2(acid, actype: str="B744", origlat: float=52., origlon: float=4., 
        destlat: float = 51., destlon: float = 3., achdg: float=None, acalt: float=0,  
        acspd: float = 0, prio: int = 1):
    """CREM2 acid,actype,origlat,origlon,[destlat],[destlon],[achdg],[acalt,[acspd],
       [prio]"""    
    
    # TODO: CREM2 with start and end nodes instead of lat lon
    
    # First, correct some values.
    acspd *= kts
    acalt *= ft

    # give path plan information about origin/destination before calling bs.traf.create
    path_plans.origin = (origlat, origlon)
    path_plans.destination = (destlat, destlon)

    # Then create the aircraft
    bs.traf.cre(acid, actype, origlat, origlon, achdg, acalt, acspd)
    
    # Add the necessary stack commands for this aircraft
    stack.stack(f'LNAV {acid} ON')
    stack.stack(f'VNAV {acid} ON')

@stack.command
def dis2int(acid: 'txt'):
    """dis2int acid"""
    # distance to next intersection
    idx = traf.id2idx(acid)

    current_edge = edge_traffic.actedge.wpedgeid[idx]

    node_id = int(current_edge.split("-",1)[1])

    node_lat, node_lon = osmid_to_latlon(current_edge, 1)

    _, d = geo.qdrdist(traf.lat[idx], traf.lon[idx], node_lat, node_lon)
    
    bs.scr.echo(f'{acid} is {d*nm} meters from node {node_id}')

@stack.command
def streetsenable():
    """streetsenable"""
    # # Turns on streets for scenario
    global streets_bool

    streets_bool = True

@stack.command
def edgeid(acid: 'txt'):
    """EDGEID, acid"""
    # check edge id of aircraft
    idx = traf.id2idx(acid)
    bs.scr.echo(f'{acid} flying over {edge_traffic.actedge.wpedgeid[idx]}')

######################## WAYPOINT TRAFFIC TRACKING  ##########################

# "traffic" class. Contains edge "autopilot" and "activedge"
class EdgeTraffic(Entity):

    def __init__(self, graph_dir):
        super().__init__()

        with self.settrafarrays():

            self.edgeap   = EdgesAp()
            self.actedge  = ActiveEdge()
        
        # Opening edges.JSON as a dictionary
        with open(f'{graph_dir}edges.json', 'r') as filename:
            self.edge_dict = json.load(filename)

        # Opening nodes.JSON as a dictionary
        with open(f'{graph_dir}nodes.json', 'r') as filename:
            self.node_dict = json.load(filename)

        # Build mapping from node to edges
        edge_array_df = pd.DataFrame(self.edge_dict).T
        self.edge_to_uv_array = np.array([(x.split('-')[0], x.split('-')[1]) for x in list(edge_array_df.index)], dtype='uint16,uint16')
        self.const_edge_stroke_array = np.array(edge_array_df['stroke_group']).astype(np.int16)
        self.const_edge_flow_array = np.array(edge_array_df['flow_group']).astype(np.int16)
        self.const_edge_speed_limit_array = np.array(edge_array_df['speed_limit']).astype(np.int16)

        # build the edge_id_array
        self.const_edge_id_array = np.arange(len(self.edge_to_uv_array), dtype='uint16')

        # intialize an empty matrix that is the len of nodes
        adj_matrix = np.zeros((len(self.node_dict), len(self.node_dict)), dtype='uint16')

        # populate this matrix (row=node_id, col=node_id) and value at row, col is edge_id
        for j, ids in enumerate(self.edge_to_uv_array):
            adj_matrix[ids[0], ids[1]] = j
        
        # convert to sparse matrix to get the (u,v) to edge_id mapping
        self.uv_to_edge_matrix = csr_matrix(adj_matrix, dtype=np.uint16)

        # assign to self to bs.traf.edgetraffic
        bs.traf.edgetraffic = self

# "autopilot"
class EdgesAp(Entity):
    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            self.edge_rou = []

    
    def create(self, n=1):
        super().create(n)

        for ridx, acid in enumerate(bs.traf.id[-n:]):
            self.edge_rou[ridx - n] = Route_edge(acid)
    
    def update_route(self, acid, acidx):

        self.edge_rou[acidx] = Route_edge(acid)

        traf.ap.route[acidx].delrte(acidx)
        
    def update(self):
        
        # Main autopilot update loop
        if bs.traf.ntraf == 0:
            return

        # See if waypoints have reached their destinations

        
        for i in bs.traf.ap.idxreached:

            edge_traffic.actedge.wpedgeid[i], edge_traffic.actedge.turn[i], \
            edge_traffic.actedge.intersection_lat[i] , edge_traffic.actedge.intersection_lon[i], \
            edge_traffic.actedge.group_number[i], edge_traffic.actedge.flow_number[i] = self.edge_rou[i].getnextwp()

            # check speed limit for this edge
            u,v = edge_traffic.actedge.wpedgeid[i].split('-')
            speed_limit = bs.traf.TrafficSpawner.graph.get_edge_data(int(u), int(v), 0)['speed_limit']
            edge_traffic.actedge.speed_limit[i] = speed_limit

            # log actual route taken
            bs.traf.TrafficSpawner.travelled_route[i].append(edge_traffic.actedge.wpedgeid[i])

        # TODO: only calculate for drones that are in constrained airspace
        # get distance of drones to next intersection
        dis_to_int = geo.kwikdist_matrix(traf.lat, traf.lon, 
                                                    edge_traffic.actedge.intersection_lat, 
                                                    edge_traffic.actedge.intersection_lon)
        
        # flatten numpy arrays
        edge_traffic.actedge.dis_to_int = np.asarray(dis_to_int).flatten()

        # print(edge_traffic.actedge.speed_limit)
        # print(bs.traf.selspd)
        # print('---------')

        return


# active edge class "the active waypoint"
class ActiveEdge(Entity):
    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            # TODO: choose correct dtypes for optimization
            self.wpedgeid = np.array([], dtype="S22")

            self.turn = np.array([], dtype=bool)

            # cosntraint airspace information
            self.intersection_lat = np.array([])
            self.intersection_lon = np.array([])

            # Distances to next intersection/turn intersection
            self.dis_to_int = np.array([])

            self.group_number = np.array([], dtype=int)
            self.flow_number = np.array([], dtype=int)

            # speed limit
            self.speed_limit = np.array([], dtype=float)

    
    def create(self, n=1):
        super().create(n)

        self.wpedgeid[-n:]                  = ""

        self.turn[-n:]                      = False

        self.intersection_lat[-n:]          = 89.99
        self.intersection_lon[-n:]          = 89.99

        self.dis_to_int[-n:]                = 9999.9

        self.group_number[-n:]              = 999
        self.flow_number[-n:]               = 999

        self.speed_limit[-n:]               = 20*kts

# route_edge class. keeps track of when aircraft move to new edges and adds edges to stack
class Route_edge(Replaceable):

    def __init__(self, acid):
        # Aircraft id (callsign) of the aircraft to which this route belongs
        self.acid = acid
        self.nwp = 0

        # Waypoint data
        self.wpname = []

        # Current actual waypoint
        self.iactwp = -1
   
        # initialize edge id list. osmids of edge
        self.wpedgeid = []
        self.turn = []

    def addwpt(self, iac, name, wpedgeid ="", turn=False):
        """Adds waypoint an returns index of waypoint, lat/lon [deg], alt[m]"""

        # For safety
        self.nwp = len(self.wpedgeid)

        name = name.upper().strip()

        newname = Route_edge.get_available_name(
            self.wpname, name, 3)

        wpidx = self.nwp

        self.addwpt_data(False, wpidx, newname, wpedgeid, turn)

        idx = wpidx
        self.nwp += 1

        return idx

    def addwpt_data(self, overwrt, wpidx, wpname, wpedgeid, turn):
        """
        Overwrites or inserts information for a waypoint
        """
        if overwrt:
            self.wpname[wpidx]  = wpname
            self.wpedgeid[wpidx] = wpedgeid
            self.turn[wpidx] = turn

        else:
            self.wpname.insert(wpidx, wpname)
            self.wpedgeid.insert(wpidx, wpedgeid)
            self.turn.insert(wpidx, turn)


    def direct(self, idx, wpnam):
        """Set active point to a waypoint by name"""
        name = wpnam.upper().strip()
        if name != "" and self.wpname.count(name) > 0:
            wpidx = self.wpname.index(name)
            self.iactwp = wpidx

            # set edge id and intersection/turn lon lat for actedge
            edge_traffic.actedge.wpedgeid[idx] = self.wpedgeid[wpidx]
            edge_traffic.actedge.turn[idx] = self.turn[wpidx]

            # distance to next node
            edge_traffic.actedge.intersection_lat[idx], edge_traffic.actedge.intersection_lon[idx] \
                = osmid_to_latlon(self.wpedgeid[wpidx], 1)

            # set group_number/flow number and edge layer_dict
            edge_traffic.actedge.group_number[idx] = edge_traffic.edge_dict[self.wpedgeid[wpidx]]['stroke_group']
            edge_traffic.actedge.flow_number[idx] = edge_traffic.edge_dict[self.wpedgeid[wpidx]]['flow_group']

            return True
        else:
            return False, "Waypoint " + wpnam + " not found"

    def getnextwp(self):
        """Returns information of next waypoint"""

        idx = bs.traf.id2idx(self.acid)
        self.iactwp = bs.traf.ap.route[idx].iactwp

        # get next edge id
        wpedgeid = self.wpedgeid[self.iactwp]
        turn = self.turn[self.iactwp]

        # get lat/lon of next intersection or distnace to constrained airspace
        intersection_lat ,intersection_lon = osmid_to_latlon(wpedgeid, 1)

        # Update group number/flow number a
        group_number = edge_traffic.edge_dict[wpedgeid]['stroke_group']
        flow_number = edge_traffic.edge_dict[wpedgeid]['flow_group']

        return wpedgeid, turn, intersection_lat, intersection_lon, group_number, flow_number

    @staticmethod
    def get_available_name(data, name_, len_=2):
        """
        Check if name already exists, if so add integer 01, 02, 03 etc.
        """
        appi = 0  # appended integer to name starts at zero (=nothing)

        fmt_ = "{:0" + str(len_) + "d}"

        # Avoid using call sign without number
        if bs.traf.id.count(name_) > 0:
            appi = 1
            name_ = name_+fmt_.format(appi)

        while data.count(name_) > 0 :
            appi += 1
            name_ = name_[:-len_]+fmt_.format(appi)
        return name_
    
def osmid_to_latlon(osmid , i=2):

    # input an edge and get the lat lon of one of the nodes
    # i = 0 gets nodeid of first node of edges
    # i = 1 gets nodeid of second node of edge

    if not i == 2:
        # if given an edge
        node_id = osmid.split("-",1)[i]
    else:
        # if given a node
        node_id = osmid

    node_lat,node_lon = edge_traffic.node_dict[node_id]

    return node_lat, node_lon

######################### PATH PLANNING #######################

class PathPlans(Entity):
    def __init__(self, graph_dir):
        super().__init__()
        
        with self.settrafarrays():
            self.pathplanning = []

        # read in graph with networkx from graphml
        self.graph = ox.load_graphml(f'{graph_dir}updated.graphml')
        self.node_gdf, self.edge_gdf = ox.graph_to_gdfs(self.graph)
        
        # get a projected dataframe of the graph
        self.node_gdf_proj = self.node_gdf.to_crs(epsg=32633)
        self.edge_gdf_proj = self.edge_gdf.to_crs(epsg=32633)
        
        # make a KDtree for the nodes
        self.kdtree = KDTree(self.node_gdf_proj[["x", "y"]])

        # create the variables for origin and destination
        self.origin = (0.0, 0.0)
        self.destination = (0.0, 0.0)

            
    def create(self, n = 1):
        super().create(n)

        self.pathplanning[-n:] = (0,)


    def plan_path(self) -> None:
        
        # todo: CREM2 with nearest nodes
        orig_node = ox.nearest_nodes(self.graph, self.node_gdf_proj, self.origin[1], self.origin[0])
        dest_node = ox.nearest_nodes(self.graph, self.node_gdf_proj, self.destination[1], self.destination[0])
        node_route = nx.shortest_path(self.graph, orig_node, dest_node, method='dijkstra')

        # get lat and lon from route and turninfo
        lats, lons, edges, _ = pluginutils.lat_lon_from_nx_route(self.graph, node_route)
        turn_bool, turn_speed, turn_coords = pluginutils.get_turn_arrays(lats, lons)
        
        # lat lon route
        route = list(zip(lats, lons))

        # get initial bearing
        qdr, _ = geo.qdrdist(lats[0], lons[0], lats[1], lons[1])
        
        return route, turn_bool, edges, turn_coords, turn_speed, qdr
