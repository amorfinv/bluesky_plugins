"""
Airspace edge information. Emulates traffic, autopilot, route, activewaypoint.
Also includes layering information.
"""

import json
import numpy as np
from collections import Counter
import pandas as pd
from scipy.sparse import csr_matrix
from scipy.spatial import KDTree
import networkx as nx

import bluesky as bs
from bluesky import core, stack, traf
from bluesky.tools.aero import ft, kts, nm
from bluesky.tools import geo
from bluesky.core import Entity, Replaceable
from bluesky.traffic import Route

from plugins.streets import sutils

bs.settings.set_variable_defaults(
    graph_dir=f'plugins/streets/graphs/rotterdam/')

# defaults
use_flow_control = True

# create classes
edge_traffic = None
flight_layers = None
path_plans = None

def init_plugin():
    
    global path_plans, edge_traffic, flight_layers

    # Initialize EdgeTraffic
    edge_traffic = EdgeTraffic(bs.settings.graph_dir)

    # Initialize Flight Layers
    flight_layers = FlightLayers(bs.settings.graph_dir)

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

        # update layer tracking
        flight_layers.layer_tracking()

####################### RESET FUNCTION ###############################

def reset():
    # when reseting bluesky turn off streets
    global streets_bool
    streets_bool = False

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
def DELRTEM2(acidx: 'acid' = None):

    # add edge info to stack
    edge_traffic.edgeap.update_route(acidx)
    return True

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
            node_dict = json.load(filename)

        # reverse dictionary
        self.node_dict = {v: k for k, v in node_dict.items()}

        # TODO: could comment everything below this line if we don't use it
        # read constrained node dict
        with open(f'{graph_dir}constrained_node_dict.json', 'r') as filename:
            constrained_node_dict = json.load(filename)

        # open constrained edges JSON as a dictionary
        with open(f'{graph_dir}const_edges.json', 'r') as filename:
            constrained_edges_dict = json.load(filename)

        # Build mapping from node to edges
        edge_array_df = pd.DataFrame(constrained_edges_dict).T
        self.edge_to_uv_array = np.array([(x.split('-')[0], x.split('-')[1]) for x in list(edge_array_df.index)], dtype='uint16,uint16')
        self.const_edge_stroke_array = np.array(edge_array_df['stroke_group']).astype(np.int16)
        self.const_edge_flow_array = np.array(edge_array_df['flow_group']).astype(np.int16)
        self.const_edge_height_allocation_array = np.array(edge_array_df['height_allocation']).astype(np.int8)        
        self.const_edge_speed_limit_array = np.array(edge_array_df['speed_limit']).astype(np.int16)

        # build the edge_id_array
        self.const_edge_id_array = np.arange(len(self.edge_to_uv_array), dtype='uint16')

        # intialize an empty matrix that is the len of nodes
        adj_matrix = np.zeros((len(constrained_node_dict), len(constrained_node_dict)), dtype='uint16')

        # populate this matrix (row=node_id, col=node_id) and value at row, col is edge_id
        for j, ids in enumerate(self.edge_to_uv_array):
            adj_matrix[ids[0], ids[1]] = j
        
        # convert to sparse matrix to get the (u,v) to edge_id mapping
        self.uv_to_edge_matrix = csr_matrix(adj_matrix, dtype=np.uint16)

# "autopilot"
class EdgesAp(Entity):
    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            self.edge_rou = []
        
        # make a vectorized function to check speed limits
        self.update_speed_limits = np.vectorize(self.check_speed_limits)
    
    def create(self, n=1):
        super().create(n)

        for ridx, acid in enumerate(bs.traf.id[-n:]):
            self.edge_rou[ridx - n] = Route_edge(acid)
    
    def update_route(self, idx):

        acid = traf.id[idx]

        self.edge_rou[idx] = Route_edge(acid)

        traf.ap.route[idx].delrte(idx)
        
    def update(self):
        
        # Main autopilot update loop

        # See if waypoints have reached their destinations
        for i in np.where(bs.traf.ap.idxreached)[0]:

            edge_traffic.actedge.wpedgeid[i], \
            edge_traffic.actedge.intersection_lat[i] , edge_traffic.actedge.intersection_lon[i], \
            edge_traffic.actedge.group_number[i], edge_traffic.actedge.flow_number[i], \
            edge_traffic.actedge.edge_layer_dict[i], \
            edge_traffic.actedge.turn_lat[i], edge_traffic.actedge.turn_lon[i], \
            edge_traffic.actedge.edge_airspace_type[i] = self.edge_rou[i].getnextwp()      

        # TODO: only calculate for drones that are in constrained airspace
        # get distance of drones to next intersection/turn intersection
        dis_to_int = geo.kwikdist_matrix(traf.lat, traf.lon, 
                                                    edge_traffic.actedge.intersection_lat, 
                                                    edge_traffic.actedge.intersection_lon)
        dis_to_turn = geo.kwikdist_matrix(traf.lat, traf.lon, 
                                                    edge_traffic.actedge.turn_lat, 
                                                    edge_traffic.actedge.turn_lon)
        
        # # check for speed limit changes
        if bs.traf.ntraf > 0:
            edge_traffic.actedge.speed_limit = self.update_speed_limits(edge_traffic.actedge.wpedgeid, 
                                                                        bs.traf.type)

        # flatten numpy arrays
        edge_traffic.actedge.dis_to_int = np.asarray(dis_to_int).flatten()
        edge_traffic.actedge.dis_to_turn = np.asarray(dis_to_turn).flatten()

        return

    @staticmethod
    def check_speed_limits(wpedgeid, ac_type):

        # check for speed limit changes
        speed_limit = int(edge_traffic.edge_dict[wpedgeid]['speed_limit'])

        # get aircraft cruise speed from last two values of ac_type
        cruise_speed = int(ac_type[-2:])

        # choose the minimum of the two for the new speed limit
        speed_limit = min(speed_limit, cruise_speed)*kts

        return speed_limit

# active edge class "the active waypoint"
class ActiveEdge(Entity):
    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            # TODO: choose correct dtypes for optimization
            self.wpedgeid = np.array([], dtype="S22")
            self.nextwpedgeid = np.array([], dtype=str)
            self.nextturnnode = np.array([], dtype=str)

            # cosntraint airspace information
            self.intersection_lat = np.array([])
            self.intersection_lon = np.array([])

            self.turn_lat = np.array([])
            self.turn_lon = np.array([])
            
            # Distances to next intersection/turn intersection
            self.dis_to_int = np.array([])
            self.dis_to_turn = np.array([])

            self.dis_to_hdg = np.array([])
            self.dis_to_const = np.array([])

            self.group_number = np.array([], dtype=int)
            self.flow_number = np.array([], dtype=int)

            # Open to optimization!
            self.edge_layer_dict = np.array([], dtype=object)

            self.edge_airspace_type = np.array([], dtype=str)

            # speed limit
            self.speed_limit = np.array([], dtype=np.int32)

    
    def create(self, n=1):
        super().create(n)

        self.wpedgeid[-n:]                  = ""
        self.nextwpedgeid[-n:]              = ""
        self.nextturnnode[-n:]              = ""

        self.intersection_lat[-n:]          = 89.99
        self.intersection_lon[-n:]          = 89.99

        self.turn_lat[-n:]                  = 89.99
        self.turn_lon[-n:]                  = 89.99

        self.dis_to_int[-n:]                = 9999.9
        self.dis_to_turn[-n:]               = 9999.9

        self.dis_to_hdg[-n:]                = 9999.9
        self.dis_to_const[-n:]              = 9999.9

        self.group_number[-n:]              = 999
        self.flow_number[-n:]               = 999
        self.edge_layer_dict[-n:]           = {}

        self.edge_airspace_type[-n:]        = ""

        self.speed_limit[-n:]               = 999

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

        # initialize location of turn in constrained airspae
        self.turn_lat = []
        self.turn_lon = []

        # initialize group_number and flow_number
        self.group_number = []
        self.flow_number = []

        # initialize edge_layer_dict
        self.edge_layer_dict = []

        # initialize the airspace type
        self.edge_airspace_type = []

    def addwpt(self, iac, name, wpedgeid ="", group_number="", flow_number="", edge_layer_dict ="", action_lat=48.1351, action_lon=11.582, edge_airspace_type="open"):
        """Adds waypoint an returns index of waypoint, lat/lon [deg], alt[m]"""

        # For safety
        self.nwp = len(self.wpedgeid)

        name = name.upper().strip()

        newname = Route_edge.get_available_name(
            self.wpname, name, 3)

        wpidx = self.nwp

        self.addwpt_data(False, wpidx, newname, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type)

        idx = wpidx
        self.nwp += 1

        return idx

    def addwpt_data(self, overwrt, wpidx, wpname, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type):
        """
        Overwrites or inserts information for a waypoint
        """
        # Process the type of action lat and lon
        turn_lat = action_lat
        turn_lon = action_lon

        if overwrt:
            self.wpname[wpidx]  = wpname
            self.wpedgeid[wpidx] = wpedgeid
            self.group_number[wpidx] = group_number
            self.flow_number[wpidx] = flow_number
            self.edge_layer_dict[wpidx] = edge_layer_dict   
            self.turn_lat[wpidx] = action_lat
            self.turn_lon[wpidx] = action_lon
            self.edge_airspace_type[wpidx] = edge_airspace_type

        else:
            self.wpname.insert(wpidx, wpname)
            self.wpedgeid.insert(wpidx, wpedgeid)
            self.group_number.insert(wpidx, group_number)
            self.flow_number.insert(wpidx, flow_number)
            self.edge_layer_dict.insert(wpidx, edge_layer_dict)
            self.turn_lat.insert(wpidx, turn_lat)
            self.turn_lon.insert(wpidx, turn_lon)
            self.edge_airspace_type.insert(wpidx, edge_airspace_type)

    def direct(self, idx, wpnam):
        """Set active point to a waypoint by name"""
        name = wpnam.upper().strip()
        if name != "" and self.wpname.count(name) > 0:
            wpidx = self.wpname.index(name)
            self.iactwp = wpidx

            # set edge id and intersection/turn lon lat for actedge
            edge_traffic.actedge.wpedgeid[idx] = self.wpedgeid[wpidx]
            
            # distance to next node
            edge_traffic.actedge.intersection_lat[idx], edge_traffic.actedge.intersection_lon[idx] \
                = osmid_to_latlon(self.wpedgeid[wpidx], 1)

            # distance to next turn
            edge_traffic.actedge.turn_lat[idx] = self.turn_lat[wpidx]
            edge_traffic.actedge.turn_lon[idx] = self.turn_lon[wpidx]

            # set group_number/flow number and edge layer_dict
            edge_traffic.actedge.group_number[idx] = self.group_number[wpidx]
            edge_traffic.actedge.flow_number[idx] = self.flow_number[wpidx]
            edge_traffic.actedge.edge_layer_dict[idx] = self.edge_layer_dict[wpidx]

            # set edge airspace type
            edge_traffic.actedge.edge_airspace_type[idx] = self.edge_airspace_type[wpidx]

            return True
        else:
            return False, "Waypoint " + wpnam + " not found"

    def getnextwp(self):
        """Returns information of next waypoint"""

        idx = bs.traf.id2idx(self.acid)
        self.iactwp = bs.traf.ap.route[idx].iactwp

        # get airspace type
        edge_airspace_type = self.edge_airspace_type[self.iactwp]

        # get next edge id
        wpedgeid = self.wpedgeid[self.iactwp]

        # get lat/lon of next intersection or distnace to constrained airspace
        intersection_lat ,intersection_lon = osmid_to_latlon(wpedgeid, 1)

        # update turn lat and lon
        turn_lat = self.turn_lat[self.iactwp]
        turn_lon = self.turn_lon[self.iactwp]

        # Update group number/flow number and edge_layer_dict
        group_number = self.group_number[self.iactwp]
        flow_number = self.flow_number[self.iactwp]
        edge_layer_dict = self.edge_layer_dict[self.iactwp]

        return wpedgeid, intersection_lat, intersection_lon, group_number, flow_number,\
               edge_layer_dict, turn_lat, turn_lon, edge_airspace_type

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
        node_id = int(osmid.split("-",1)[i])
    else:
        # if given a node
        node_id = int(osmid)

    node_latlon = edge_traffic.node_dict[node_id]
    node_lat = float(node_latlon.split("-",1)[0])
    node_lon = float(node_latlon.split("-",1)[1])

    return node_lat, node_lon
######################## FLIGHT LAYER TRACKING ############################

class FlightLayers(Entity):
    def __init__(self, graph_dir):
        super().__init__()

        with self.settrafarrays():
            self.flight_levels                  = np.array([], dtype=int)
            self.flight_layer_type              = np.array([], dtype=str)

            self.closest_cruise_layer_bottom    = np.array([], dtype=int)
            self.closest_cruise_layer_top       = np.array([], dtype=int)

            self.closest_turn_layer_bottom      = np.array([], dtype=int)
            self.closest_turn_layer_top         = np.array([], dtype=int)
            
            self.closest_empty_layer_bottom     = np.array([], dtype=int)
            self.closest_empty_layer_top        = np.array([], dtype=int)

            self.lowest_cruise_layer            = np.array([], dtype=int)
            self.highest_cruise_layer           = np.array([], dtype=int)
            
            self.ac_angle_levels                = np.array([], dtype=float)
            self.leg_angle_levels               = np.array([], dtype=float)
            self.next_leg_angle_levels          = np.array([], dtype=float)

            self.open_closest_layer             = np.array([], dtype=int)


        self.layer_dict = {}

        # Opening edges.JSON as a dictionary
        with open(f'{graph_dir}layers.json', 'r') as filename:
            self.layer_dict = json.load(filename)
        
        self.layer_spacing = self.layer_dict['info']['spacing']
        self.angle_spacing = self.layer_dict['info']['angle_spacing']
        self.layer_levels = np.array(self.layer_dict['info']['levels'])
        self.layer_dist_center = self.layer_levels - self.layer_dict['info']['spacing']/2

        # vectorize function to extract layer_info
        self.layer_info = np.vectorize(self.get_layer_type)
    
    def create(self, n=1):
        super().create(n)

        self.flight_levels[-n:]                 = 0
        self.flight_layer_type[-n:]             = ""

        self.closest_cruise_layer_top[-n:]      = 0
        self.closest_cruise_layer_bottom[-n:]   = 0

        self.closest_turn_layer_top[-n:]        = 0
        self.closest_turn_layer_bottom[-n:]     = 0
        
        self.closest_empty_layer_top[-n:]       = 0
        self.closest_empty_layer_bottom[-n:]    = 0

        self.lowest_cruise_layer[-n:]           = 0
        self.highest_cruise_layer[-n:]          = 0
        
    def layer_tracking(self):
        
        # update flight levels
        self.flight_levels = np.array((np.round((bs.traf.alt/ft) / self.layer_spacing))*self.layer_spacing, dtype=int)
        
        self.flight_levels = np.where(self.flight_levels < 0, 30, self.flight_levels)
        self.flight_levels = np.where(self.flight_levels > 480, 480, self.flight_levels)
        # update flight layer type
        edge_layer_dicts = edge_traffic.actedge.edge_layer_dict

        # only go into vectorized function if there is traffic
        if bs.traf.ntraf > 0:
            self.flight_layer_type, self.closest_cruise_layer_bottom, self.closest_cruise_layer_top,\
            self.closest_turn_layer_bottom, self.closest_turn_layer_top,\
            self.closest_empty_layer_bottom, self.closest_empty_layer_top,\
            self.lowest_cruise_layer, self.highest_cruise_layer = self.layer_info(self.flight_levels, edge_layer_dicts, 
                                                        edge_traffic.actedge.edge_airspace_type)

        return

    @staticmethod
    def get_layer_type(flight_level, edge_layer_dict, airspace_type):

        # print("print",flight_level,edge_layer_dict, airspace_type )
        # vectorized function to process edge layer dictionary
        # TODO: maybe access dictionaries here instead of having it
        # as active waypoint data

        # get correct layer_info_list based on your flight level
        layer_list = edge_layer_dict[f'{flight_level}']
        
        # get layer_type
        layer_type = layer_list[0]

        # get closest cruise layers
        closest_cruise_layer_bottom = layer_list[1]
        closest_cruise_layer_top = layer_list[2]

        if closest_cruise_layer_bottom == '':
            closest_cruise_layer_bottom = 0

        if closest_cruise_layer_top =='':
            closest_cruise_layer_top = 0

        # get closest turnlayers
        closest_turn_layer_bottom = layer_list[3]
        closest_turn_layer_top = layer_list[4]

        if closest_turn_layer_bottom == '':
            closest_turn_layer_bottom = 0

        if closest_turn_layer_top =='':
            closest_turn_layer_top = 0
            
        # get closest empty layers
        closest_empty_layer_bottom = layer_list[5]
        closest_empty_layer_top = layer_list[6]

        if closest_empty_layer_bottom == '':
            closest_empty_layer_bottom = 0

        if closest_empty_layer_top =='':
            closest_empty_layer_top = 0

        # get idx lowest cruise layer and highest
        idx_lowest_cruise_layer, idx_highest_cruise_layer = 7, 8
        
        lowest_cruise_layer = layer_list[idx_lowest_cruise_layer]

        if lowest_cruise_layer == '':
            lowest_cruise_layer = 0
        
        # get highest cruise layer
        highest_cruise_layer = layer_list[idx_highest_cruise_layer]

        if highest_cruise_layer == '':
            highest_cruise_layer = 0

        return layer_type, closest_cruise_layer_bottom, closest_cruise_layer_top, \
            closest_turn_layer_bottom, closest_turn_layer_top, \
            closest_empty_layer_bottom, closest_empty_layer_top, \
            lowest_cruise_layer, highest_cruise_layer

######################### PATH PLANNING #######################

class PathPlans(Entity):
    def __init__(self, graph_dir):
        super().__init__()
        
        with self.settrafarrays():
            self.pathplanning = []

        # read in graph with networkx from graphml
        self.graph = sutils.load_graphml(f'{graph_dir}graph.graphml')
        self.node_gdf, self.edge_gdf = sutils.graph_to_gdfs(self.graph)
        
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

        acid = bs.traf.id[-1]
        ridx = -1
        
        route, turns, edges, next_turn, turn_speeds, qdr = self.plan_path()

        # set initial bearing
        bs.traf.hdg[-1] = qdr
        bs.traf.trk[-1] = qdr
        
        # Get needed values
        acrte = Route._routes.get(acid)

        for j, rte in enumerate(route):
            lat = rte[0] # deg
            lon = rte[1] # deg
            alt = -999
            spd = -999
            
            # Do flyby or flyturn processing
            if turns[j]:
                acrte.turnspd = turn_speeds[j]*kts
                acrte.swflyby   = False
                acrte.swflyturn = True
            else:
                # Either it's a flyby, or a typo.
                acrte.swflyby   = True
                acrte.swflyturn = False
            
            name    = acid
            wptype  = Route.wplatlon
            
            acrte.addwpt_simple(ridx, name, wptype, lat, lon, alt, spd)
        
            # Add the streets stuff            
            wpedgeid = edges[j]
            
            group_number = edge_traffic.edge_dict[wpedgeid]['height_allocation']
            edge_layer_type = edge_traffic.edge_dict[wpedgeid]['height_allocation']
            edge_layer_dict = flight_layers.layer_dict["config"][edge_layer_type]['levels']
            group_number = edge_traffic.edge_dict[wpedgeid]['stroke_group']
            flow_number = edge_traffic.edge_dict[wpedgeid]['flow_group']

            # get the edge_airspace_type
            edge_airspace_type = 'open'

            turn_lat = next_turn[j][1]
            turn_lon = next_turn[j][0]
            edge_traffic.edgeap.edge_rou[ridx].addwpt(ridx, name, wpedgeid, group_number, flow_number, edge_layer_dict, 
                                        turn_lat, turn_lon, edge_airspace_type)

        # For this aircraft, manually set the first "next_qdr" in actwp
        # We basically need to find the qdr between the second and the third waypoint, as
        # the first one is the origin
        if len(acrte.wplat)>2:
            bs.traf.actwp.next_qdr[ridx], _ = geo.qdrdist(acrte.wplat[1], acrte.wplon[1],
                                                        acrte.wplat[2], acrte.wplon[2])
            
        # Calculate flight plan
        acrte.calcfp()
        edge_traffic.edgeap.edge_rou[ridx].direct(ridx,edge_traffic.edgeap.edge_rou[ridx].wpname[1])

        self.pathplanning[-1] = ...


    def plan_path(self) -> None:
        
        # todo: CREM2 with nearest nodes
        orig_node = sutils.nearest_nodes(self.kdtree, self.node_gdf_proj, self.origin[1], self.origin[0])
        dest_node = sutils.nearest_nodes(self.kdtree, self.node_gdf_proj, self.destination[1], self.destination[0])
        node_route = nx.shortest_path(self.graph, orig_node, dest_node, method='dijkstra')

        # get lat and lon from route and turninfo
        lats, lons, edges, _ = sutils.lat_lon_from_nx_route(self.graph, node_route)
        turn_bool, turn_speed, turn_coords = sutils.get_turn_arrays(lats, lons)
        
        # lat lon route
        route = list(zip(lats, lons))

        # get initial bearing
        qdr, _ = geo.qdrdist(lats[0], lons[0], lats[1], lons[1])
        
        return route, turn_bool, edges, turn_coords, turn_speed, qdr