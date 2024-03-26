import numpy as np
from scipy.cluster.vq import whiten
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon, Point
import json
import geopandas as gpd
import pandas as pd
from pyproj import Transformer
import matplotlib.pyplot as plt
from collections import defaultdict

import bluesky as bs
from bluesky.core.simtime import timed_function
from bluesky import core
from bluesky.tools import datalog
from bluesky.tools.aero import nm
from bluesky.tools import areafilter as af
from bluesky.stack import command, stack
from plugins.clusters import cluster_algorithms as calgos

clusterheader = \
    '#######################################################\n' + \
    'Cluster log LOG\n' + \
    'Cluster density statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Simulation time [s], ' + \
    'Conflict count [-], ' + \
    'Geometry [wkt], ' + \
    'Summed edged length [m], ' + \
    'Cluster linear densities  [m], ' + \
    'Density category [-]\n'

clustering = None

def init_plugin():
    global clustering

    # Addtional initilisation code
    bs.traf.clustering = Clustering()
    # Configuration parameters
    config = {
        'plugin_name':     'CONFLICTCLUSTERING',
        'plugin_type':     'sim',
        'reset': reset
    }

    return config

def reset():
    bs.traf.clustering.reset()

class Clustering(core.Entity):
    def __init__(self):
        super().__init__() 

        # genereate the transformers
        self.transformer_to_utm    = Transformer.from_crs("EPSG:4326", "EPSG:28992")
        self.transformer_to_latlon = Transformer.from_crs("EPSG:28992", "EPSG:4326")

        self.polygons_to_draw = []
        self.draw_the_polygons = False

        self.distance_threshold = 2000

        # make observation time to look at past data
        self.observation_time = 10*60

        # minimum number of aircraft to be considered a cluster
        self.min_ntraf = 3

        # polygon data
        self.cluster_polygons = None

        # log cluster data
        self.clusterlog = datalog.crelog('CLUSTERLOG', None, clusterheader)

        # edges to check for replanning
        self.cluster_edges = []

        # set the weights of the graph
        self.low_density_weight = 1.0
        self.medium_density_weight = 1.5
        self.high_density_weight = 2.0

        # load the density dictionary
        with open(f'{bs.settings.plugin_path}/clusters/densityjsons/conflicttraffic.json', 'r') as file:
            # Load the JSON data into a dictionary
            self.density_dictionary = json.load(file)

        self.scen_density_dict = {}

        # set the density cutoff
        self.low_density_cutoff = '0.25'
        self.medium_density_cutoff = '0.50'

        with self.settrafarrays():
            self.cluster_labels = np.array([])

    def reset(self):

        # genereate the transformers
        self.transformer_to_utm    = Transformer.from_crs("EPSG:4326", "EPSG:28992")
        self.transformer_to_latlon = Transformer.from_crs("EPSG:28992", "EPSG:4326")

        self.polygons_to_draw = []
        self.draw_the_polygons = False

        self.distance_threshold = 2000

        # make observation time to look at past data
        self.observation_time = 10*60

        # minimum number of aircraft to be considered a cluster
        self.min_ntraf = 3

        # polygon data
        self.cluster_polygons = None

        # log cluster data
        self.clusterlog = datalog.crelog('CLUSTERLOG', None, clusterheader)

        # edges to check for replanning
        self.cluster_edges = []

        # set the weights of the graph
        self.low_density_weight = 1.0
        self.medium_density_weight = 1.5
        self.high_density_weight = 2.0

        # load the density dictionary
        with open(f'{bs.settings.plugin_path}/clusters/densityjsons/conflicttraffic.json', 'r') as file:
            # Load the JSON data into a dictionary
            self.density_dictionary = json.load(file)

        self.scen_density_dict = {}
        
        # set the density cutoff
        self.low_density_cutoff = '0.25'
        self.medium_density_cutoff = '0.50'

        with self.settrafarrays():
            self.cluster_labels = np.array([])

    def create(self, n=1):
        super().create(n)
        self.cluster_labels[-n:] = -1

    def delete_polygons(self):
        if not self.draw_the_polygons:
            return
        # first delete the areas from bluesky screen
        for areaname, *_ in self.polygons_to_draw:            
            af.deleteArea(areaname)
        
        self.polygons_to_draw = []

    @timed_function(dt=10)
    def clustering(self):

        # delete polygons in screen
        self.delete_polygons()

        if bs.traf.ntraf == 0:
           return
        
        if bs.sim.simt <= 600:
           return
        
        # First convert the aircraft positions to meters
        x,y = self.transformer_to_utm.transform(bs.traf.lat, bs.traf.lon)
        # define a geoseries
        point_list = [Point(xp,yp) for xp,yp in zip(x,y)]
        point_geoseries = gpd.GeoSeries(point_list, crs='EPSG:28992')
        
        # filter the conflict dictionary to remove items from more than observation time
        keys_to_remove = []
        for past_time in bs.traf.cd.conf_cluster:
            if  bs.sim.simt - float(past_time) > self.observation_time:
                keys_to_remove.append(past_time)
        
        for past_time in keys_to_remove:
            bs.traf.cd.conf_cluster.pop(past_time)

        if len(bs.traf.cd.conf_cluster) == 0:
            return

        # make the observation matrix from the conflicts
        lat_lon_confs = np.vstack(list(bs.traf.cd.conf_cluster.values()))
        x,y = self.transformer_to_utm.transform(lat_lon_confs[:,0],lat_lon_confs[:,1])
        features = np.column_stack((x, y))
        
        if len(features) == 0:
            return
        
        # here we order the observations for peace of mind and round
        features = features[np.argsort(features[:, 0])]
        # features = np.round(features)

        # normalize the observation matrix
        whitened = whiten(features)

        # do clustering and return the optimal labels
        optimal_labels = calgos.ward(features, self.distance_threshold)

        # get number of clusters
        n_clusters = np.max(optimal_labels)+1

        # polygonize cluster the clusters
        polygons = self.polygonize(optimal_labels, features, n_clusters)
        
        if polygons.empty:
            return
    
        # now we want to find out which edges intersect with the polygons and update streets plugin
        # with the new flow group numbers
        new_edges, polygons = self.edges_intersect(polygons, point_geoseries)

        # calculate densities of the cluster areas
        polygons = self.calc_densities(polygons, new_edges)

        # TODO: apply flow control rules so not all clusters need to replan
        # self.cluster_polygons, self.cluster_edges = self.apply_density_rules(polygons, new_edges)
        self.cluster_polygons, self.cluster_edges = self.apply_density_rules_easy(polygons, new_edges)

        # cluster logginf
        self.update_logging()

        # draw the polygons
        self.draw_polygons()

        # code to plot in matplotlib
        # self.external_plotting(features, optimal_labels)

    def aircraft_intersect(self, polygons, point_geoseries):

        # this function checks which aircraft are in which cluster
        # query the polygon and points intersections
        intersections = point_geoseries.sindex.query(polygons['geometry'], predicate='intersects')
        
        unique_values, unique_indices = np.unique(intersections[1], return_index=True)
        intersections_poly = intersections[0][unique_indices]
        intersections_confs = intersections[1][unique_indices]
        # i
        # now assign the polygon values to the cluster labels
        mask = np.zeros_like(self.cluster_labels, dtype=bool)
        mask[intersections_confs] = True
        
        self.cluster_labels[mask] = intersections_poly
        self.cluster_labels[~mask] = -1

    def apply_density_rules(self, polygons, edges_df):

        # anything below this cutoff is low density and above is medium density
        low_linear_density = self.scen_density_dict[ self.low_density_cutoff]
        # anything above this cutoff is high density
        medium_linear_density = self.scen_density_dict[self.medium_density_cutoff]

        # Categorize the density into three categories
        polygons['density_category'] = pd.cut(polygons['conf_linear_density'],
                                            bins=[float('-inf'), low_linear_density, medium_linear_density, float('inf')],
                                            labels=['low', 'medium', 'high'])

        # add these categories to the edges_df        
        merged_df = pd.merge(polygons, edges_df, left_index=True, right_on='flow_group', how='left')

        # Apply conditions based on 'density_category'
        merged_df['adjusted_length'] = merged_df.apply(lambda row: row['length'] * self.medium_density_weight if row['density_category'] == 'medium' 
                                                                        else (row['length'] * self.high_density_weight if row['density_category'] == 'high' 
                                                                                else (row['length'] * self.low_density_weight)), axis=1)
        # update the TrafficSpawner graph
        # # Update edge attributes in the graph
        cluster_edge_lengths = {row.Index: row.adjusted_length for row in merged_df.itertuples()}
        # also get the edges of the complete graph
        full_edges = {row.Index: row.length for row in edges_df.itertuples()}
        
        # reset the lengths of the traffic spawner graph. These are the original graph lengths
        for edge_label, length in full_edges.items():
            bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length'] = length 

        # apply the adjusted lengths to the graph
        for edge_label, adjusted_length in cluster_edge_lengths.items():
            old_length =  bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length']
            new_length = adjusted_length
            bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length'] = adjusted_length 

        # select indices of edges in the medium or high category
        selected_indices = merged_df[merged_df['density_category'].isin(['medium', 'high'])].index
        selected_indices = [f'{u}-{v}' for u,v,_ in selected_indices]

        # save polygons to draw later
        if self.draw_the_polygons:
            polygon_colors = {'low': 'green', 'medium': 'blue', 'high': 'red'}
            for polygon in polygons.itertuples():
                
                # if polygon.density_category == 'low':
                #     continue
                labels = polygon.flow_group
                polygon_geom = polygon.geometry
                polygon_color = polygon_colors[polygon.density_category]
                
                self.polygons_to_draw.append((f'CLUSTER{labels}', polygon_geom, polygon_color))
        
        return polygons, selected_indices
    
    def apply_density_rules_easy(self, polygons, edges_df):

        if len(polygons) == 1:
            polygons['density_category'] = 'high'
        else:
            # apply easy quantiles
            polygons['density_category'] = pd.qcut(polygons['conf_linear_density'], q=[0, 0.25, 0.5, 1], labels=['low', 'medium', 'high'])

        # add these categories to the edges_df        
        merged_df = pd.merge(polygons, edges_df, left_index=True, right_on='flow_group', how='left')

        # Apply conditions based on 'density_category'
        merged_df['adjusted_length'] = merged_df.apply(lambda row: row['length'] * self.medium_density_weight if row['density_category'] == 'medium' 
                                                                        else (row['length'] * self.high_density_weight if row['density_category'] == 'high' 
                                                                                else (row['length'] * self.low_density_weight)), axis=1)
        # update the TrafficSpawner graph
        # # Update edge attributes in the graph
        cluster_edge_lengths = {row.Index: row.adjusted_length for row in merged_df.itertuples()}
        # also get the edges of the complete graph
        full_edges = {row.Index: row.length for row in edges_df.itertuples()}
        
        # reset the lengths of the traffic spawner graph. These are the original graph lengths
        for edge_label, length in full_edges.items():
            bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length'] = length 

        # apply the adjusted lengths to the graph
        for edge_label, adjusted_length in cluster_edge_lengths.items():
            old_length =  bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length']
            new_length = adjusted_length
            bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length'] = adjusted_length 

        # select indices of edges in the medium or high category
        selected_indices = merged_df[merged_df['density_category'].isin(['medium', 'high'])].index
        selected_indices = [f'{u}-{v}' for u,v,_ in selected_indices]

        # save polygons to draw later
        if self.draw_the_polygons:
            polygon_colors = {'low': 'green', 'medium': 'blue', 'high': 'red'}
            for polygon in polygons.itertuples():
                
                # if polygon.density_category == 'low':
                #     continue
                labels = polygon.flow_group
                polygon_geom = polygon.geometry
                polygon_color = polygon_colors[polygon.density_category]
                
                self.polygons_to_draw.append((f'CLUSTER{labels}', polygon_geom, polygon_color))
        
        return polygons, selected_indices

    def calc_densities(self, polygons, new_edges):
        # TODO: use COINS on each individual polygon cluster?

        # get linear length of each edge that is part of flow group
        grouped_lengths = new_edges.groupby('flow_group')['length'].sum()
        grouped_lengths = grouped_lengths.drop(-1)
        
        # sort both to make sure it is in correct order
        grouped_lengths = grouped_lengths.sort_index()
        polygons = polygons.sort_index()

        # add the lengths as a column
        polygons['edge_length'] = grouped_lengths

        # calculate the linear density
        polygons['conf_linear_density'] = polygons['conf_count'] / polygons['edge_length'] * 1000

        # calculate the area density
        polygons['conf_area_density'] = polygons['conf_count'] / polygons['geometry'].area * 1000000

        return polygons
    
    def edges_intersect(self, polygons, point_geoseries):

        # Check for intersections, Note this returns integer indices so use .iloc
        poly_intersection_indices, edge_intersection_indices = bs.traf.TrafficSpawner.edges.sindex.query_bulk(polygons['geometry'], predicate="intersects")

        # convert to regular index which is .loc
        poly_indices = polygons.iloc[poly_intersection_indices].index.to_numpy()
        edge_indices = bs.traf.TrafficSpawner.edges.iloc[edge_intersection_indices].index
        edge_index_strings = [f'{u}-{v}' for u, v, key in edge_indices]

        # create a geoseries using the poly indices to merge it with the original edge_gdf
        new_series = pd.Series(poly_indices, index=edge_indices, dtype=int)

        # Check if there are repeated values in the edge_indices, if yes then an edge is part of several polygons with .iloc
        has_repeated_values = len(np.unique(edge_intersection_indices)) < len(edge_intersection_indices)
        if has_repeated_values:

            # if there are repeated values then we select the one in which the length of the edge is highest

            # here keeep a note of which edge indices intersect multiple clusters get these in .loc format
            repeated_dict = defaultdict(list)
            for edge_value, polygon_value in zip(edge_intersection_indices, poly_intersection_indices):
                key = bs.traf.TrafficSpawner.edges.iloc[edge_value].name
                value = polygons.iloc[polygon_value].name
                repeated_dict[key].append(value)
            repeat_edges = {key: value for key, value in repeated_dict.items() if len(value) > 1}

            # get polygons to check
            polygons_with_repeats = [item for sublist in repeat_edges.values() for item in sublist]
            polygons_with_repeats = list(set(polygons_with_repeats))

            # here check which polygon has the largest part of the edge
            selected_polygons = {key: None for key in repeat_edges.keys()}
            for edge_value, polygon_value in repeat_edges.items():
                # get the edge geometry
                edge_geometry = bs.traf.TrafficSpawner.edges.loc[edge_value].geometry
                intersection_lengths = []
                for poly_intersect in polygon_value:
                    # get polygon geometry
                    polygon_geometry = polygons.loc[poly_intersect].geometry
                    # get length of intersection
                    intersection_length = edge_geometry.intersection(polygon_geometry).length
                    intersection_lengths.append((poly_intersect, intersection_length))
                
                # here select the polygon that contains the highest length of the edge
                max_intersection = max(intersection_lengths, key=lambda x: x[1])

                # choose this polygon for the given edge
                selected_polygons[edge_value] = max_intersection[0]

            # now we update the new series based on repeated polygons 
            # remove the duplicates
            new_series = new_series[~new_series.index.duplicated(keep='first')]

            # assign correct value to the series
            for duplicate_edge, polygon_choice in selected_polygons.items():
                u,v,key = duplicate_edge
                new_series[u,v,key] = polygon_choice

            # get these values for the updated datfarme as they are used to update the graph
            poly_indices = new_series.to_list()
            edge_indices = new_series.index.to_list()
            edge_index_strings = [f'{u}-{v}' for u, v, _ in edge_indices] 

        # merge the dataframes and create a new one with the flow_group info
        new_edges_df = pd.merge(
            bs.traf.TrafficSpawner.edges, 
            new_series.to_frame(name='flow_group'), 
            left_index=True, 
            right_index=True,
            how='left')
        
        new_edges_df.fillna(-1, inplace=True)
        new_edges_df['flow_group'] = new_edges_df['flow_group'].astype(int)
        new_edges_df['length'] =  new_edges_df['geometry'].apply(lambda x: x.length)

        # After checking if an edge is part of several polygons it may be possible that
        # a polygon is left without any edges
        check_assigned_flow_groups = np.sort(new_edges_df['flow_group'].unique())
        check_assigned_flow_groups = check_assigned_flow_groups[1:]

        get_polygon_flow_groups = np.array(polygons['flow_group'])

        # check if there are any missing polygons in assigned flow groups
        unassigned_groups = np.setdiff1d(get_polygon_flow_groups, check_assigned_flow_groups)

        # remove value from polygons
        polygons = polygons[~polygons['flow_group'].isin(unassigned_groups)]

        # here check which aircraft fall within the polygons
        self.aircraft_intersect(polygons, point_geoseries)

        # next step is to update the flow_number in the "streets" plugin.
        # update the edge_traffic dictionary
        for key, value in bs.traf.edgetraffic.edge_dict.items():
            if key in edge_index_strings:
                index = edge_index_strings.index(key)
                value["flow_group"] = poly_indices[index]
            else:
                value["flow_group"] = -1   


        #  Update the active edges as well
        for idx, _ in enumerate(bs.traf.id):
            edgeid = bs.traf.edgetraffic.actedge.wpedgeid[idx]

            if self.cluster_labels[idx] >= 0:
                bs.traf.edgetraffic.actedge.flow_number[idx] = bs.traf.edgetraffic.edge_dict[edgeid]['flow_group']
            else:
                bs.traf.edgetraffic.actedge.flow_number[idx] = -1
        
        return new_edges_df, polygons
  
    def polygonize(self, optimal_labels, features, n_clusters, buffer_dist=bs.settings.asas_pzr):
        
        # loop through all of the clusters and create a polygon
        polygon_data = {'flow_group': [], 'geometry': [], 'conf_count': []}
         
        for optimal_label in range(n_clusters):
            label_mask = optimal_labels == optimal_label
            cluster_points = features[label_mask,:]

            # ensure you only keep unique coordinates
            cluster_points = np.unique(cluster_points, axis=0)
            
            # skip if less than minimum
            if cluster_points.shape[0] <= self.min_ntraf:
                    continue
            hull = ConvexHull(cluster_points)
            polygon = Polygon(cluster_points[hull.vertices])
            polygon = polygon.buffer(buffer_dist*nm)

            polygon_data['flow_group'].append(optimal_label)
            polygon_data['geometry'].append(polygon)
            polygon_data['conf_count'].append(cluster_points.shape[0])

        # create geodataframe of polygons
        poly_gdf = gpd.GeoDataFrame(polygon_data, index=polygon_data['flow_group'], crs='EPSG:28992')

        # set the lavels as the index
        # # check for potential intersections
        # *_,intersections = poly_gdf.sindex.query_bulk(poly_gdf['geometry'], predicate="intersects")
        # if len(intersections) != len(poly_gdf):
        #      print('self intersecting polygons')
        
        return poly_gdf

    def draw_polygons(self):

        if not self.draw_the_polygons:
            return

        # draw the polygons
        for areaname, polygon, colour in self.polygons_to_draw:            
            lat, lon = self.transformer_to_latlon.transform(*polygon.exterior.coords.xy)
            coordinates = [x for pair in zip(lat, lon) for x in pair]
            af.defineArea(areaname, 'POLY', coordinates)

            # set the color
            stack(f'COLOUR {areaname} {colour}')

    def external_plotting(self, features, optimal_labels):
        for _, polygon in self.polygons_to_draw: 
            plt.plot(polygon.exterior.xy[0],polygon.exterior.xy[1])   

        plt.scatter(features[:, 0], features[:, 1], c=optimal_labels)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Optimal Clustering')
        plt.show()        

    def update_logging(self):
        
        if len(self.cluster_polygons) == 0:
            return

        conf_count = self.cluster_polygons['conf_count']
        geometry = self.cluster_polygons['geometry']
        edge_length = self.cluster_polygons['edge_length']
        linear_densities = self.cluster_polygons['conf_linear_density']
        density_category = self.cluster_polygons['density_category']

        self.clusterlog.log(*conf_count)
        self.clusterlog.log(*geometry)
        self.clusterlog.log(*edge_length)
        self.clusterlog.log(*linear_densities)
        self.clusterlog.log(*density_category)

    @command 
    def STARTCLUSTERLOG(self):
        self.clusterlog.start()
    
    @command 
    def SETCLUSTERDISTANCE(self, dist:int):
        self.distance_threshold=dist

        # also set the cluster densut dict as this is final stack command
        target_ntraf = bs.traf.TrafficSpawner.target_ntraf

        try:
            self.scen_density_dict = self.density_dictionary[str(target_ntraf)][str(dist)]
        except KeyError:
            pass

    @command 
    def SETOBSERVATIONTIME(self, time:int):
        self.observation_time = time
    
    @command 
    def SETGRAPHWEIGHTS(self,low_density_weight:float, medium_density_weight:float, high_density_weight:float):
        # set the weights of the graph
        self.low_density_weight = low_density_weight
        self.medium_density_weight = medium_density_weight
        self.high_density_weight = high_density_weight

    @command 
    def SETDENSITYCUTOFF(self, medium_density_cutoff:str, high_density_cutoff:str):
        # set the cutoff
        self.low_density_cutoff = medium_density_cutoff
        self.medium_density_cutoff = high_density_cutoff

    @command 
    def DRAWPOLYGONS(self):
        # set the cutoff
        self.draw_the_polygons = True
    