import numpy as np
from scipy.cluster.vq import whiten
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import osmnx as ox
import geopandas as gpd
import pandas as pd
from pyproj import Transformer
import matplotlib.pyplot as plt
from collections import Counter

import bluesky as bs
from bluesky.core.simtime import timed_function
from bluesky import core
from bluesky.tools import datalog
from bluesky.tools.aero import nm
from bluesky.tools import areafilter as af
from bluesky.stack import command
from plugins.clusters import cluster_algorithms as calgos

clusterheader = \
    '#######################################################\n' + \
    'Cluster log LOG\n' + \
    'Cluster density statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Simulation time [s], ' + \
    'Aircraft count [-], ' + \
    'Geometry [wkt], ' + \
    'Summed edged length [m], ' + \
    'Cluster linear densities  [m], ' + \
    'Cluster area densities [-]\n'

clustering = None

def init_plugin():
    global clustering

    # Addtional initilisation code
    bs.traf.clustering = Clustering()
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'CLUSTERING',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

class Clustering(core.Entity):
    def __init__(self):
        super().__init__() 

        # genereate the transformers
        self.transformer_to_utm    = Transformer.from_crs("EPSG:4326", "EPSG:28992")
        self.transformer_to_latlon = Transformer.from_crs("EPSG:28992", "EPSG:4326")

        self.polygons_to_draw = []
        self.draw_the_polygons = False

        self.distance_threshold = 3000

        # make observation time to look at past data
        self.observation_time = 10*60

        # case for live traffic clustering
        self.cluster_case = 'livetraffic'

        # minimum number of aircraft to be considered a cluster
        self.min_ntraf = 2

        # polygon data
        self.cluster_polygons = None

        # log cluster data
        self.clusterlog = datalog.crelog('CLUSTERDENSITIES', None, clusterheader)

        # edges to check for replanning
        self.cluster_edges = []

        with self.settrafarrays():
            self.cluster_labels = np.array([])
  
    def create(self, n=1):
        super().create(n)
        self.cluster_labels[-n:] = -1


    @timed_function(dt=10)
    def delete_polygons(self):
        if not self.draw_the_polygons:
            return
        # first delete the areas from bluesky screen
        for areaname, _ in self.polygons_to_draw:            
            af.deleteArea(areaname)
        
        self.polygons_to_draw = []

    @timed_function(dt=10)
    def clustering(self):

        # first step is to access the streets plugin
        edge_traffic = bs.traf.edgetraffic
        features = np.array([])

        if bs.traf.ntraf < 2:
            return

        if self.cluster_case == 'livetraffic':
            # First convert the aircraft positions to meters and make observation matrix
            x,y = self.transformer_to_utm.transform(bs.traf.lat, bs.traf.lon)
            features = np.column_stack((x, y))

            # distance thresold for ward clustering
            # a working cluster distance is 3000
            distance_threshold = 3000

        elif self.cluster_case == 'conflicts':

            # a working cluster distance is 2000

            # make observation matrix with conflicts from the past ten minutes
            observation_time = 10*60
            # filter the conflict dictionary to remove items from more than ten minutes ago
            keys_to_remove = []
            for past_time in bs.traf.cd.conf_cluster:
                if  bs.sim.simt - float(past_time) > self.observation_time:
                    keys_to_remove.append(past_time)
            
            for past_time in keys_to_remove:
                bs.traf.cd.conf_cluster.pop(past_time)
            
            # make the observation matrix from the conflicts
            if len(bs.traf.cd.conf_cluster) > 1:
                lat_lon_confs = np.vstack(list(bs.traf.cd.conf_cluster.values()))
                x,y = self.transformer_to_utm.transform(lat_lon_confs[:,0],lat_lon_confs[:,1])
                features = np.column_stack((x, y))
                
                distance_threshold = 2000

        elif self.cluster_case == 'intrusions':

            # make observation matrix with intrusions from the past ten minutes
            observation_time = 10*60
            # filter the conflict dictionary to remove items from more than ten minutes ago
            keys_to_remove = []
            for past_time in bs.traf.cd.los_cluster:
                if  bs.sim.simt - float(past_time) > self.observation_time:
                    keys_to_remove.append(past_time)
            
            for past_time in keys_to_remove:
                bs.traf.cd.los_cluster.pop(past_time)
            
            # make the observation matrix from the conflicts
            if len(bs.traf.cd.los_cluster) > 1:
                lat_lon_los = np.vstack(list(bs.traf.cd.los_cluster.values()))
                x,y = self.transformer_to_utm.transform(lat_lon_los[:,0],lat_lon_los[:,1])
                features = np.column_stack((x, y))
                
                distance_threshold = 2000

        if len(features) == 0:
            return

        # normalize the observation matrix
        whitened = whiten(features)

        # do k-means clustering and return the optimal labels
        optimal_labels = calgos.ward(features, self.distance_threshold)
        
        # get number of clusters
        n_clusters = np.max(optimal_labels)+1

        # polygonize cluster the clusters
        polygons = self.polygonize(optimal_labels, features, n_clusters)
        
        # get numpy array of aircraft with relevant clusters
        self.cluster_labels = np.where(
            np.isin(
                optimal_labels, 
                [item for item, count in Counter(optimal_labels).items() if count > self.min_ntraf]
                ), 
            optimal_labels, 
            -1
                )
        
        # now we want to find out which edges intersect with the polygons and update streets plugin
        # with the new flow group numbers
        new_edges = self.edges_intersect(polygons, edge_traffic)

        # calculate densities of the cluster areas
        polygons = self.calc_densities(polygons, edge_traffic, new_edges)

        # TODO: apply flow control rules so not all clusters need to replan
        self.cluster_polygons, self.cluster_edges = self.apply_density_rules(polygons, new_edges)

        # cluster logginf
        self.update_logging()

        # code to plot in matplotlib
        # self.external_plotting(features, optimal_labels)

    def apply_density_rules(self, polygons, edges_df):
        

        # TODO: run for a while to check density levels
        # Three density levels
        low_linear_density = 3
        medium_linear_density = 4.5
        high_linear_density = 6


        # Categorize the density into three categories
        polygons['density_category'] = pd.cut(polygons['ac_linear_density'],
                                            bins=[float('-inf'), low_linear_density, medium_linear_density, float('inf')],
                                            labels=['low', 'medium', 'high'])
        

        # add these categories to the edges_df        
        merged_df = pd.merge(polygons, edges_df, left_index=True, right_on='flow_group', how='left')

        # Apply conditions based on 'density_category'
        merged_df['adjusted_length'] = merged_df.apply(lambda row: row['length'] * 1.5 if row['density_category'] == 'medium' 
                                                                        else (row['length'] * 2 if row['density_category'] == 'high' 
                                                                                else (row['length'])), axis=1)

        # update the TrafficSpawner graph
        # TODO: combine these two for loops into one

        # # Update edge attributes in the graph
        edge_lengths = {row.Index: row.adjusted_length for row in merged_df.itertuples()}

        for edge_label, adjusted_length in edge_lengths.items():
            bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length'] = adjusted_length 

        # select indices of edges in the medium or high category
        selected_indices = merged_df[merged_df['density_category'].isin(['medium', 'high'])].index
        selected_indices = [f'{u}-{v}' for u,v,key in selected_indices]
        
        return polygons, selected_indices

    def calc_densities(self, polygons,edge_traffic, new_edges):
        # TODO: only for livetraffic currently
        # Calcualte the densities
        flow_count_dict = dict(Counter(edge_traffic.actedge.flow_number))

        # TODO: perform COINS on each individual polygon cluster?

        # get linear length of each edge that is part of flow group
        grouped_lengths = new_edges.groupby('flow_group')['length'].sum()
        grouped_lengths = grouped_lengths.drop(-1)
        
        # sort both to make sure it is in correct order
        grouped_lengths = grouped_lengths.sort_index()
        polygons = polygons.sort_index()

        # add the lengths as a column
        polygons['edge_length'] = grouped_lengths

        # add the aircraft counts
        polygons['ac_count'] = polygons.index.map(flow_count_dict)

        # calculate the linear density
        polygons['ac_linear_density'] = polygons['ac_count'] / polygons['edge_length'] * 10000

        # calculate the area density
        polygons['ac_area_density'] = polygons['ac_count'] / polygons['geometry'].area * 1000000

        return polygons
    
    def edges_intersect(self, polygons, edge_traffic):

        # Check for intersections, Note this returns integer indices
        poly_indices, edge_indices = bs.traf.TrafficSpawner.edges.sindex.query_bulk(polygons['geometry'], predicate="intersects")

        # Check if there are repeated values in the edge_indices, if yes then an edge is part of several polygons
        # TODO: fix these case
        has_repeated_values = len(np.unique(edge_indices)) < len(edge_indices)

        # if has_repeated_values:
        #     print("The array has repeated values.")

        # convert to regular index
        poly_indices = polygons.iloc[poly_indices].index.to_numpy()
        edge_indices = bs.traf.TrafficSpawner.edges.iloc[edge_indices].index
        edge_index_strings = [f'{u}-{v}' for u, v, key in edge_indices]

        # create a geoseries using the poly indices to merge it with the original edge_gdf
        new_series = pd.Series(poly_indices, index=edge_indices, dtype=int)

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

        # next step is to update the flow_number in the "streets" plugin.
        # update the edge_traffic dictionary
        for key, value in edge_traffic.edge_dict.items():
            if key in edge_index_strings:
                index = edge_index_strings.index(key)
                value["flow_group"] = poly_indices[index]
            else:
                value["flow_group"] = -1   


        #  Update the active edges as well
        for idx, _ in enumerate(bs.traf.id):
            edgeid = edge_traffic.actedge.wpedgeid[idx]

            if self.cluster_labels[idx] >= 0:
                edge_traffic.actedge.flow_number[idx] = edge_traffic.edge_dict[edgeid]['flow_group']
            else:
                edge_traffic.actedge.flow_number[idx] = -1
        
        return new_edges_df

            
    def polygonize(self, optimal_labels, features, n_clusters, buffer_dist=bs.settings.asas_pzr*4):
        
        # loop through all of the clusters and create a polygon
        polygon_data = {'flow_group': [], 'geometry': []}

        for optimal_label in range(n_clusters):
            label_mask = optimal_labels == optimal_label
            cluster_points = features[label_mask,:]

            # skip if less than minimum
            if cluster_points.shape[0] <= self.min_ntraf:
                    continue
            hull = ConvexHull(cluster_points)
            polygon = Polygon(cluster_points[hull.vertices])
            polygon = polygon.buffer(buffer_dist*nm)

            polygon_data['flow_group'].append(optimal_label)
            polygon_data['geometry'].append(polygon)

            # save polygons to draw later
            if self.draw_the_polygons:
                self.polygons_to_draw.append((f'CLUSTER{optimal_label}', polygon))

        # create geodataframe of polygons
        poly_gdf = gpd.GeoDataFrame(polygon_data, index=polygon_data['flow_group'], crs='EPSG:28992')

        # set the lavels as the index
        # # check for potential intersections
        # *_,intersections = poly_gdf.sindex.query_bulk(poly_gdf['geometry'], predicate="intersects")
        # if len(intersections) != len(poly_gdf):
        #      print('self intersecting polygons')

        
        return poly_gdf

    @timed_function(dt=10)
    def draw_polygons(self):

        if not self.draw_the_polygons:
            return
        # draw the polygons
        for areaname, polygon in self.polygons_to_draw:            
            lat, lon = self.transformer_to_latlon.transform(*polygon.exterior.coords.xy)
            coordinates = [x for pair in zip(lat, lon) for x in pair]
            af.defineArea(areaname, 'POLY', coordinates)

    def external_plotting(self, features, optimal_labels):
        for _, polygon in self.polygons_to_draw: 
            plt.plot(polygon.exterior.xy[0],polygon.exterior.xy[1])   

        plt.scatter(features[:, 0], features[:, 1], c=optimal_labels)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Optimal Clustering')
        plt.show()        

    def update_logging(self):
        
        ac_count = self.cluster_polygons['ac_count']
        geometry = self.cluster_polygons['geometry']
        edge_length = self.cluster_polygons['edge_length']
        linear_densities = self.cluster_polygons['ac_linear_density']
        area_densities = self.cluster_polygons['ac_area_density']

        self.clusterlog.log(*ac_count)
        self.clusterlog.log(*geometry)
        self.clusterlog.log(*edge_length)
        self.clusterlog.log(*linear_densities)
        self.clusterlog.log(*area_densities)

        pass

    @command 
    def STARTLOG(self):
        self.clusterlog.start()
        return
    
    @command 
    def SETCLUSTERDISTANCE(self, dist:int):
        
        self.distance_threshold=dist
        
        return

    
    @command 
    def SETCLUSTERCASE(self, clustercase:str):
        
        self.cluster_case = clustercase
        
        return