import numpy as np
import geopandas as gpd
import osmnx as ox
import os
import pickle
import random
from copy import deepcopy

import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
from bluesky import stack
from bluesky.tools.geo import kwikqdrdist, kwikdist_matrix, qdrpos, kwikdist
from bluesky.tools.aero import kts, ft, fpm, nm
from bluesky.traffic import Route
from bluesky.tools.misc import degto180

from plugins.utils import pluginutils

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'SyntheticTrafficSpawner',
        'plugin_type': 'sim',
        'reset': reset
    }
    # Put TrafficSpawner in bs.traf
    bs.traf.TrafficSpawner = SyntheticTrafficSpawner()
    return config

def reset():
    bs.traf.TrafficSpawner.reset()

class SyntheticTrafficSpawner(Entity):
    def __init__(self):
        super().__init__()
        self.target_ntraf = 100
        self.synthetic_zone = 2
        # Load default city
        self.graph, self.original_graph, self.edges, self.nodes = self.loadcity('synthetic')
        # Traffic ID increment
        self.traf_id = 1
        #default alt and speed
        self.alt = 100 * ft
        self.spd = 20 * kts
        # When to stop simulating
        self.stop_time = 7*24*60*60
        self.stop_time_enable = True
        # Number of conflicts when to stop simulating
        self.stop_conf = 10000
        self.stop_conf_enable = False
        # Turn ASAS on
        # stack.stack('ASAS ON')
        # # Set a default seed
        #stack.stack('SEED 12345')
        
        # Logging related stuff
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.confinside_all = 0
        self.deleted_aircraft = 0
        self.losmindist = dict()
        
        self.planned_routes = dict()
        
        with self.settrafarrays():
            self.route_edges = []
            self.unique_edges = []
            self.original_route = []
            self.route_nodes = []
            self.travelled_route = []

            # Metrics
            self.distance2D = np.array([])
            self.distance3D = np.array([])
            self.distancealt = np.array([])
            self.create_time = np.array([])

            # missiontype
            self.mission_type = np.array([], dtype=str)
        return
    
    def create(self, n=1):
        super().create(n)
        # Store creation time of new aircraft
        self.route_edges[-n:] = [0]*n # Default edge
        self.unique_edges[-n:] = [0]*n # Default edge
        self.original_route[-n:] = [0]*n
        self.route_nodes[-n:] = [0]*n
        self.travelled_route[-n:] = [0]*n

        self.distance2D[-n:] = [0]*n
        self.distance3D[-n:] = [0]*n
        self.distancealt[-n:] = [0]*n
        self.create_time[-n:] = [0]*n

        self.mission_type[-n:] = ''
    
    def reset(self):
        self.synthetic_zone = 2
        self.target_ntraf = 100
        # Load default city
        self.graph, self.original_graph, self.edges, self.nodes = self.loadcity('synthetic')
        # Traffic ID increment
        self.traf_id = 1
        #default alt and speed
        self.alt = 100 * ft
        self.spd = 20 * kts
        # When to stop simulating
        self.stop_time = 7*24*60*60
        self.stop_time_enable = True
        # Number of conflicts when to stop simulating
        self.stop_conf = 10000
        self.stop_conf_enable = False
        # Turn ASAS on
        #stack.stack('ASAS ON')
        # Set a default seed
        #stack.stack('SEED 12345')
        
        # Logging related stuff
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.confinside_all = 0
        self.deleted_aircraft = 0
        self.losmindist = dict()

        # get stuff for logging routes
        self.planned_routes = dict()
        

        with self.settrafarrays():
            self.route_edges = []
            self.unique_edges = []
            self.original_route = []
            self.route_nodes = []
            self.travelled_route = []

            # Metrics
            self.distance2D = np.array([])
            self.distance3D = np.array([])
            self.distancealt = np.array([])
            self.create_time = np.array([])

            # mssion type
            self.mission_type = np.array([], dtype=str)


    @command
    def loadcity(self, city = None):
        # Get a list of available cities
        list_of_cities = [x for x in os.listdir(f'plugins/scenario_maker/') if '.py' not in x]
        if city == None or city not in list_of_cities:
            bs.scr.echo(f'The following cities are available: {list_of_cities}.')
            return
        self.city = city
        self.path = f'plugins/scenario_maker/{self.city}'
        self.load_origins_destinations()
        # Set the origin point of the city
        with open(f'{self.path}/centre.txt', 'r') as f:
            coords = f.readlines()
        self.city_centre_coords = [float(coords[0]), float(coords[1])]
        # Load the graph for the city
        # read gpkgs that are
        nodes = gpd.read_file(f'{self.path}/updated.gpkg', layer='nodes')
        edges = gpd.read_file(f'{self.path}/updated.gpkg', layer='edges')

        # set the indices 
        edges.set_index(['u', 'v', 'key'], inplace=True)
        nodes.set_index(['osmid'], inplace=True)

        # ensure that it has the correct value
        nodes['x'] = nodes['geometry'].apply(lambda x: x.x)
        nodes['y'] = nodes['geometry'].apply(lambda x: x.y)

        G = ox.graph_from_gdfs(nodes, edges)
        G_original = deepcopy(G)

        # convert both to CRS:28992
        edges_transformed = edges.to_crs('EPSG:28992')
        nodes_transformed = nodes.to_crs('EPSG:28992')

        # now add an edge length attribute
        #edges_transformed['length'] =  edges_transformed['geometry'].apply(lambda x: x.length)

        # force create spatial index
        edges_transformed.sindex
        
        return G, G_original, edges_transformed, nodes_transformed
    
    @command
    def trafficnumber(self, target_ntraf = 50):
        self.target_ntraf = int(target_ntraf)
        bs.scr.echo(f'The target traffic number was set to {target_ntraf}.')
        return
    
    @command
    def stopsimt(self, time):
        # This will be the time at which we stop and quit.
        self.stop_time = int(time)
        self.stop_time_enable = True
        self.stop_conf_enable = False
        
    @command
    def stopconf(self, confno):
        # This will be the number of conflicts at which we stop and quit.
        self.stop_conf = int(confno)
        self.stop_conf_enable = True
        self.stop_time_enable = False

    @command
    def SYNSCN(self, syntheticnumber:int):
        self.synthetic_zone = syntheticnumber
    
    def load_origins_destinations(self):
        with open(f'{self.path}/orig_dest_dict_zone{self.synthetic_zone}.pickle', 'rb') as f:
            self.orig_dest_dict = pickle.load(f)
    
    @timed_function(dt = 1)
    def spawn_traffic_zone1(self):

        if not self.synthetic_zone == 1:
            return
        # load edge traffic for use
        # TODO: check for potential replans
        streets = pluginutils.access_plugin_object('streets')
        
        '''Function to spawn traffic to maintain a traffic level equal to ntraf.'''
        attempts = 0

        while bs.traf.ntraf < self.target_ntraf and attempts < 20:
            # Choose a random origin and destination
            origin = random.choice(list(self.orig_dest_dict.keys()))
            destination = random.choice(self.orig_dest_dict[origin])
            # Load the pickle file for that
            with open(f'{self.path}/pickles/conflictzone{self.synthetic_zone}/{origin}-{destination}.pkl', 'rb') as f:
                pickled_route = pickle.load(f)
                
            # This pickle route has LAT, LON, EDGE, TURN. Unpack em
            lats, lons, edges, turns = list(zip(*pickled_route))
            # Check if any other aircraft is too close to the origin
            dist = kwikdist_matrix(np.array([lats[0]]), np.array([lons[0]]), bs.traf.lat, bs.traf.lon)

            # Second check, if distance is smaller than rpz * 4?
            if np.any(dist<(bs.settings.asas_pzr*1)):
                # Try again with another random aircraft
                attempts += 1
                continue
            
            # We are successful
            attempts = 0
            
            # Obtain required data for aircraft
            acid = f'D{self.traf_id}'
            self.traf_id += 1
            actype = 'M600'
            achdg, _ = kwikqdrdist(lats[0], lons[0], lats[1], lons[1])
            
            # Let's create the aircraft
            bs.traf.cre(acid, actype, lats[0], lons[0], achdg, self.alt, 20*kts)
            
            # Get more info
            acrte = Route._routes.get(acid)
            acidx = bs.traf.id.index(acid)
            
            # Add the edges to this guy
            self.route_edges[acidx] = edges

            # now create a unique edges dictionary. Doint in this weird manner so that
            # I am able to persever order
            # TODO: maybe use stroke groups
            unique_edges = list({f'{u}-{v}':None for u,v in edges}.keys())
            self.unique_edges[acidx] = np.array(unique_edges)
            self.original_route[acidx] = deepcopy(unique_edges)
            self.planned_routes[acid] = []
            self.travelled_route[acidx] = []

            # list of unique nodes
            seen = set()
            seen_add = seen.add
            self.route_nodes[acidx] = [edge[0] for edge in edges if not (edge[0] in seen or seen_add(edge[0]))]
            self.route_nodes[acidx].append(edges[-1][1])

            # Start adding waypoints
            for edgeid, lat, lon, turn in zip(edges, lats, lons, turns):
                if turn:
                    acrte.turnspd = 5 * kts
                    acrte.swflyby = False
                    acrte.swflyturn = True
                else:
                    acrte.swflyby = True
                    acrte.swflyturn = False
                    
                wptype  = Route.wplatlon
                acrte.addwpt_simple(acidx, acid, wptype, lat, lon, self.alt, self.spd)

                # Add the streets stuff
                edgeidx = '-'.join(map(str, edgeid))

                streets.edge_traffic.edgeap.edge_rou[acidx].addwpt(acidx, acid, edgeidx, turn)
            
            # Calculate the flight plan
            acrte.calcfp()
            streets.edge_traffic.edgeap.edge_rou[acidx].direct(acidx,streets.edge_traffic.edgeap.edge_rou[acidx].wpname[1])

            # Turn lnav on for this aircraft
            stack.stack(f'LNAV {acid} ON')
            stack.stack(f'VNAV {acid} ON')
            stack.stack(f'COLOUR {acid} RED')
            # save the create time
            self.create_time[acidx] = bs.sim.simt

            # save the mission type
            self.mission_type[acidx] = 'clusterzone'

    @timed_function(dt = 30)
    def spawn_through_traffic_zone_1(self):

        if not self.synthetic_zone == 1:
                return
        # load edge traffic for use
        # TODO: check for potential replans
        streets = pluginutils.access_plugin_object('streets')
        
        '''Function to spawn traffic to maintain a traffic level equal to ntraf.'''

        # Choose a random origin and destination
        # origin = 16
        # destination = 1495

        # origin, destination = 758, 1988
        origin, destination = 83, 1819

        # Load the pickle file for that
        with open(f'{self.path}/pickles/pathszone{self.synthetic_zone}/{origin}-{destination}.pkl', 'rb') as f:
            pickled_route = pickle.load(f)
            
        # This pickle route has LAT, LON, EDGE, TURN. Unpack em
        lats, lons, edges, turns = list(zip(*pickled_route))
        # Check if any other aircraft is too close to the origin
        
        # Obtain required data for aircraft
        acid = f'D{self.traf_id}'
        self.traf_id += 1
        actype = 'M600'
        achdg, _ = kwikqdrdist(lats[0], lons[0], lats[1], lons[1])
        
        # Let's create the aircraft
        bs.traf.cre(acid, actype, lats[0], lons[0], achdg, self.alt, 20*kts)
        
        # Get more info
        acrte = Route._routes.get(acid)
        acidx = bs.traf.id.index(acid)
        
        # Add the edges to this guy
        self.route_edges[acidx] = edges

        # now create a unique edges dictionary. Doint in this weird manner so that
        # I am able to persever order
        # TODO: maybe use stroke groups
        unique_edges = list({f'{u}-{v}':None for u,v in edges}.keys())
        self.unique_edges[acidx] = np.array(unique_edges)
        self.original_route[acidx] = deepcopy(unique_edges)
        self.planned_routes[acid] = []
        self.travelled_route[acidx] = []

        # list of unique nodes
        seen = set()
        seen_add = seen.add
        self.route_nodes[acidx] = [edge[0] for edge in edges if not (edge[0] in seen or seen_add(edge[0]))]
        self.route_nodes[acidx].append(edges[-1][1])

        # Start adding waypoints
        for edgeid, lat, lon, turn in zip(edges, lats, lons, turns):
            if turn:
                acrte.turnspd = 5 * kts
                acrte.swflyby = False
                acrte.swflyturn = True
            else:
                acrte.swflyby = True
                acrte.swflyturn = False
                
            wptype  = Route.wplatlon
            acrte.addwpt_simple(acidx, acid, wptype, lat, lon, self.alt, self.spd)

            # Add the streets stuff
            edgeidx = '-'.join(map(str, edgeid))

            streets.edge_traffic.edgeap.edge_rou[acidx].addwpt(acidx, acid, edgeidx, turn)
        
        # Calculate the flight plan
        acrte.calcfp()
        streets.edge_traffic.edgeap.edge_rou[acidx].direct(acidx,streets.edge_traffic.edgeap.edge_rou[acidx].wpname[1])

        # Turn lnav on for this aircraft
        stack.stack(f'LNAV {acid} ON')
        stack.stack(f'VNAV {acid} ON')
        stack.stack(f'COLOUR {acid} BLUE')

        # save the create time
        self.create_time[acidx] = bs.sim.simt
        
        self.mission_type[acidx] = 'regular'


    @timed_function(dt = 1)
    def spawn_traffic_zone2(self):

        if not self.synthetic_zone == 2:
            return
        
        # load edge traffic for use
        # TODO: check for potential replans
        streets = pluginutils.access_plugin_object('streets')
        
        paths = [
            (1837, 1576, 'conflictzone2'), 
            (19, 56, 'pathszone2')
                 ]
        
        for origin, destination, mission_type in paths:
            # Load the pickle file for that
            with open(f'{self.path}/pickles/{mission_type}/{origin}-{destination}.pkl', 'rb') as f:
                pickled_route = pickle.load(f)
                
            # This pickle route has LAT, LON, EDGE, TURN. Unpack em
            lats, lons, edges, turns = list(zip(*pickled_route))
            # Check if any other aircraft is too close to the origin

            dist = kwikdist_matrix(np.array([lats[0]]), np.array([lons[0]]), bs.traf.lat, bs.traf.lon)

            # Second check, if distance is smaller than rpz * 4?
            if np.any(dist<(bs.settings.asas_pzr*3)):
                # Try again with another random aircraft
                continue
            
            
            # Obtain required data for aircraft
            acid = f'D{self.traf_id}'
            self.traf_id += 1
            actype = 'M600'
            achdg, _ = kwikqdrdist(lats[0], lons[0], lats[1], lons[1])
            
            # Let's create the aircraft
            bs.traf.cre(acid, actype, lats[0], lons[0], achdg, self.alt, 20*kts)
            
            # Get more info
            acrte = Route._routes.get(acid)
            acidx = bs.traf.id.index(acid)
            
            # Add the edges to this guy
            self.route_edges[acidx] = edges

            # now create a unique edges dictionary. Doint in this weird manner so that
            # I am able to persever order
            # TODO: maybe use stroke groups
            unique_edges = list({f'{u}-{v}':None for u,v in edges}.keys())
            self.unique_edges[acidx] = np.array(unique_edges)
            self.original_route[acidx] = deepcopy(unique_edges)
            self.planned_routes[acid] = []
            self.travelled_route[acidx] = []

            # list of unique nodes
            seen = set()
            seen_add = seen.add
            self.route_nodes[acidx] = [edge[0] for edge in edges if not (edge[0] in seen or seen_add(edge[0]))]
            self.route_nodes[acidx].append(edges[-1][1])

            # Start adding waypoints
            for edgeid, lat, lon, turn in zip(edges, lats, lons, turns):
                if turn:
                    acrte.turnspd = 5 * kts
                    acrte.swflyby = False
                    acrte.swflyturn = True
                else:
                    acrte.swflyby = True
                    acrte.swflyturn = False
                    
                wptype  = Route.wplatlon
                acrte.addwpt_simple(acidx, acid, wptype, lat, lon, self.alt, self.spd)

                # Add the streets stuff
                edgeidx = '-'.join(map(str, edgeid))

                streets.edge_traffic.edgeap.edge_rou[acidx].addwpt(acidx, acid, edgeidx, turn)

            # Calculate the flight plan
            acrte.calcfp()
            streets.edge_traffic.edgeap.edge_rou[acidx].direct(acidx,streets.edge_traffic.edgeap.edge_rou[acidx].wpname[1])

            # Turn lnav on for this aircraft
            stack.stack(f'LNAV {acid} ON')
            stack.stack(f'VNAV {acid} ON')

            if mission_type == 'conflictzone2':
                stack.stack(f'COLOUR {acid} RED')
            else:
                stack.stack(f'COLOUR {acid} BLUE')


            # save the create time
            self.create_time[acidx] = bs.sim.simt
            
    
            self.mission_type[acidx] = mission_type

    
    @timed_function(dt = bs.sim.simdt)
    def delete_aircraft(self):
        # Update logging
        self.update_logging()
        # Delete aircraft that have LNAV off and have gone past the last waypoint.
        # Also added logging in here because why not.
        lnav_on = bs.traf.swlnav
        still_going_to_dest = np.logical_and(abs(degto180(bs.traf.trk - bs.traf.ap.qdr2wp)) < 10.0, 
                                       bs.traf.ap.dist2wp > 5)
        delete_array = np.logical_and.reduce((np.logical_not(lnav_on), 
                                         bs.traf.actwp.swlastwp,
                                         np.logical_not(still_going_to_dest)))
        
        if np.any(delete_array):
            # Get the ACIDs of the aircraft to delete
            acids_to_delete = np.array(bs.traf.id)[delete_array]
            for acid in acids_to_delete:
                # Log the stuff for this aircraft in the flstlog
                idx = bs.traf.id.index(acid)
                bs.traf.CDRLogger.flst.log(
                    acid,
                    self.create_time[idx],
                    bs.sim.simt - self.create_time[idx],
                    (self.distance2D[idx]),
                    (self.distance3D[idx]),
                    (self.distancealt[idx]),
                    bs.traf.lat[idx],
                    bs.traf.lon[idx],
                    bs.traf.alt[idx]/ft,
                    bs.traf.tas[idx]/kts,
                    bs.traf.vs[idx]/fpm,
                    bs.traf.hdg[idx],
                    bs.traf.cr.active[idx],
                    bs.traf.aporasas.alt[idx]/ft,
                    bs.traf.aporasas.tas[idx]/kts,
                    bs.traf.aporasas.vs[idx]/fpm,
                    bs.traf.aporasas.hdg[idx])
                

                # get original route
                original_route = self.original_route[idx]
                origin_node = original_route[0].split('-')[0]
                destination_node = original_route[-1].split('-')[1]
                original_route = [origin_node] + [edge.split('-')[1] for edge in original_route]
                original_route = '-'.join(original_route)

                # get route travelled
                travelled_route = self.travelled_route[idx]
                unique_travelled_edges = list({edge: None for edge in travelled_route}.keys())
                travelled_route = [edge.split('-')[0] for edge in unique_travelled_edges]

                if len(self.original_route[idx]) == 1:
                    travelled_route = self.original_route[idx]

                # check if origin or destination node is part of list
                if not travelled_route[0] == origin_node:
                    travelled_route = [origin_node] + travelled_route

                if not travelled_route[-1] == destination_node:
                    travelled_route = travelled_route + [destination_node]

                travelled_route = '-'.join(travelled_route)

                # check if original is same as travelled
                is_same_route = original_route == travelled_route

                # get planned routes

                if len(self.planned_routes[acid]):
                    planned_routes = []
                    for route in self.planned_routes[acid]:
                        first_node = route[0].split('-')[0]
                        route = [first_node] + [edge.split('-')[1] for edge in route]
                        route = '-'.join(route)
                        planned_routes.append(route)

                    n_replans = len(planned_routes)

                    # now merge the routes    
                    planned_routes = ','.join(planned_routes)
                    
                else:
                    # make empty if no plans
                    planned_routes = ''
                    n_replans = 0

                # add acid to data
                original_route = f'{acid},' + original_route
                travelled_route = f'{acid},' + travelled_route
                is_same_route = f'{acid},{is_same_route}'
                n_replans = f'{acid},{n_replans}'
                planned_routes = f'{acid},' +  planned_routes
                mission_type = f'{acid},' + self.mission_type[idx]

                # log the information
                bs.traf.flowcontrol.replanlog.log(mission_type)
                bs.traf.flowcontrol.replanlog.log(original_route)
                bs.traf.flowcontrol.replanlog.log(travelled_route)
                bs.traf.flowcontrol.replanlog.log(is_same_route)
                bs.traf.flowcontrol.replanlog.log(n_replans)
                bs.traf.flowcontrol.replanlog.log(planned_routes)

                # now delete from planned routes dictionary
                del self.planned_routes[acid]

                stack.stack(f'DEL {acid}')
                
        if (self.stop_time_enable and bs.sim.simt > self.stop_time) or \
            (self.stop_conf_enable and len(bs.traf.cd.confpairs_all) > self.stop_conf):
            stack.stack(f'HOLD')
            # print(bs.traf.cd.unique_conf_id_counter)
            stack.stack(f'DELETEALL')
            stack.stack(f'RESET')
            
    def update_logging(self):        
        # Increment the distance metrics
        resultantspd = np.sqrt(bs.traf.gs * bs.traf.gs + bs.traf.vs * bs.traf.vs)
        self.distance2D += bs.sim.simdt * abs(bs.traf.gs)
        self.distance3D += bs.sim.simdt * resultantspd
        self.distancealt += bs.sim.simdt * abs(bs.traf.vs)
        
        # Now let's do the CONF and LOS logs
        confpairs_new = list(set(bs.traf.cd.confpairs) - self.prevconfpairs)
        if confpairs_new:
            done_pairs = []
            for pair in set(confpairs_new):
                # Check if the aircraft still exist
                if (pair[0] in bs.traf.id) and (pair[1] in bs.traf.id):
                    # Get the two aircraft
                    idx1 = bs.traf.id.index(pair[0])
                    idx2 = bs.traf.id.index(pair[1])
                    done_pairs.append((idx1,idx2))
                    if (idx2,idx1) in done_pairs:
                        continue
                        
                    bs.traf.CDRLogger.conflog.log(pair[0], pair[1],
                                    bs.traf.lat[idx1], bs.traf.lon[idx1],bs.traf.alt[idx1],
                                    bs.traf.lat[idx2], bs.traf.lon[idx2],bs.traf.alt[idx2])
                
        self.prevconfpairs = set(bs.traf.cd.confpairs)
        
        # Losses of separation as well
        # We want to track the LOS, and log the minimum distance and altitude between these two aircraft.
        # This gives us the lospairs that were here previously but aren't anymore
        lospairs_out = list(self.prevlospairs - set(bs.traf.cd.lospairs))
        
        # Attempt to calculate current distance for all current lospairs, and store it in the dictionary
        # if entry doesn't exist yet or if calculated distance is smaller.
        for pair in bs.traf.cd.lospairs:
            # Check if the aircraft still exist
            if (pair[0] in bs.traf.id) and (pair[1] in bs.traf.id):
                idx1 = bs.traf.id.index(pair[0])
                idx2 = bs.traf.id.index(pair[1])
                # Calculate current distance between them [m]
                losdistance = kwikdist(bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.lat[idx2], bs.traf.lon[idx2])*nm
                # To avoid repeats, the dictionary entry is DxDy, where x<y. So D32 and D564 would be D32D564
                dictkey = pair[0]+pair[1] if int(pair[0][1:]) < int(pair[1][1:]) else pair[1]+pair[0]
                if dictkey not in self.losmindist:
                    # Set the entry
                    self.losmindist[dictkey] = [losdistance, 
                                                bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.alt[idx1], 
                                                bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.alt[idx2],
                                                bs.sim.simt, bs.sim.simt]
                    # This guy here                             ^ is the LOS start time
                else:
                    # Entry exists, check if calculated is smaller
                    if self.losmindist[dictkey][0] > losdistance:
                        # It's smaller. Make sure to keep the LOS start time
                        self.losmindist[dictkey] = [losdistance, 
                                                bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.alt[idx1], 
                                                bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.alt[idx2],
                                                bs.sim.simt, self.losmindist[dictkey][8]]
        
        # Log data if there are aircraft that are no longer in LOS
        if lospairs_out:
            done_pairs = []
            for pair in set(lospairs_out):
                # Get their dictkey
                dictkey = pair[0]+pair[1] if int(pair[0][1:]) < int(pair[1][1:]) else pair[1]+pair[0]
                # Is this pair in the dictionary?
                if dictkey not in self.losmindist:
                    # Pair was already logged, continue
                    continue
                losdata = self.losmindist[dictkey]
                # Remove this aircraft pair from losmindist
                self.losmindist.pop(dictkey)
                #Log the LOS
                bs.traf.CDRLogger.loslog.log(losdata[8], losdata[7], pair[0], pair[1],
                                losdata[1], losdata[2],losdata[3],
                                losdata[4], losdata[5],losdata[6],
                                losdata[0])
                
        
        self.prevlospairs = set(bs.traf.cd.lospairs)
        
    @command
    def deleteall(self):
        '''Deletes all aircraft.'''
        while bs.traf.ntraf>0:
            bs.traf.delete(0)
        return
    
