import numpy as np
from scipy.cluster.vq import whiten
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import geopandas as gpd
import pandas as pd
from pyproj import Transformer
import matplotlib.pyplot as plt
from collections import Counter

import bluesky as bs
from bluesky.core.simtime import timed_function
from bluesky import core
from bluesky.tools.aero import nm
from bluesky.tools import areafilter as af
from plugins.clusters import cluster_algorithms as calgos
from plugins.utils import pluginutils

clustering = None

def init_plugin():
    global clustering

    # Addtional initilisation code
    clustering = Clustering()
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

        # case for live traffic clustering
        self.cluster_case = 'livetraffic'

        # minimum number of aircraft to be considered a cluster
        self.min_ntraf = 2

        with self.settrafarrays():
                
            self.cluster_labels = np.array([])
  
    def create(self, n=1):
        super().create(n)
        self.cluster_labels[-n:] = -1

    

    @timed_function(dt=10)
    def delete_polygons(self):
        # first delete the areas from bluesky screen
        for areaname, _ in self.polygons_to_draw:            
            af.deleteArea(areaname)
        
        self.polygons_to_draw = []

    @timed_function(dt=10)
    def clustering(self):
        
        features = np.array([])

        if bs.traf.ntraf < 2:
            return

        if self.cluster_case == 'livetraffic':
            # First convert the aircraft positions to meters and make observation matrix
            x,y = self.transformer_to_utm.transform(bs.traf.lat, bs.traf.lon)
            features = np.column_stack((x, y))

            # distance thresold for ward clustering
            distance_threshold = 3000

        elif self.cluster_case == 'conflicts':

            # make observation matrix with conflicts from the past ten minutes
            observation_time = 10*60
            # filter the conflict dictionary to remove items from more than ten minutes ago
            keys_to_remove = []
            for past_time in bs.traf.cd.conf_cluster:
                if  bs.sim.simt - float(past_time) > observation_time:
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
                if  bs.sim.simt - float(past_time) > observation_time:
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
        optimal_labels = calgos.ward(features, distance_threshold)
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
        self.edges_intersect(polygons)

        # code to plot in matplotlib
        # self.external_plotting(features, optimal_labels)
        
    def edges_intersect(self, polygons):

        # Check for intersections, Note this returns integer indices
        poly_indices, edge_indices = bs.traf.TrafficSpawner.edges.sindex.query_bulk(polygons['geometry'], predicate="intersects")

        # Check if there are repeated values in the edge_indices, if yes then an edge is part of several polygons
        # TODO: fix these case
        has_repeated_values = len(np.unique(edge_indices)) < len(edge_indices)

        if has_repeated_values:
            print("The array has repeated values.")

        # convert to regular index
        poly_indices = polygons.iloc[poly_indices].index.to_numpy()
        edge_indices = bs.traf.TrafficSpawner.edges.iloc[edge_indices].index
        edge_index_strings = [f'{u}-{v}' for u, v, key in edge_indices]

        # create a geoseries using the poly indices to merge it with the original edge_gdf
        new_series = pd.Series(poly_indices, index=edge_indices)

        # merge the dataframes and create a new one with the flow_group info
        new_edges = pd.merge(
            bs.traf.TrafficSpawner.edges, 
            new_series.to_frame(name='flow_group'), 
            left_index=True, 
            right_index=True,
            how='left')
        
        # now update the edge_dict in streets plugin with "flow group"
        edge_traffic = pluginutils.access_plugin_object('streets').edge_traffic

        for key, value in edge_traffic.edge_dict.items():
            if key in edge_index_strings:
                index = edge_index_strings.index(key)
                value["flow_group"] = poly_indices[index]
            else:
                value["flow_group"] = -1        


        # next step is to update the flow_number in the "streets" plugin
        for idx, _ in enumerate(bs.traf.id):
            edgeid = edge_traffic.actedge.wpedgeid[idx]

            if self.cluster_labels[idx] >= 0:

                edge_traffic.actedge.flow_number[idx] = edge_traffic.edge_dict[edgeid]['flow_group']
            else:
                edge_traffic.actedge.flow_number[idx] = -1

            

    def polygonize(self, optimal_labels, features, n_clusters, buffer_dist=bs.settings.asas_pzr*4):
        
        # loop through all of the clusters and create a polygon
        polygon_data = {'label': [], 'geometry': []}

        for optimal_label in range(n_clusters):
            label_mask = optimal_labels == optimal_label
            cluster_points = features[label_mask,:]

            # skip if less than minimum
            if cluster_points.shape[0] <= self.min_ntraf:
                    continue
            hull = ConvexHull(cluster_points)
            polygon = Polygon(cluster_points[hull.vertices])
            polygon = polygon.buffer(buffer_dist*nm)

            polygon_data['label'].append(optimal_label)
            polygon_data['geometry'].append(polygon)

            # save polygons to draw later
            self.polygons_to_draw.append((f'CLUSTER{optimal_label}', polygon))

        # create geodataframe of polygons
        poly_gdf = gpd.GeoDataFrame(polygon_data, index=polygon_data['label'], crs='EPSG:28992')

        # set the lavels as the index
        # # check for potential intersections
        # *_,intersections = poly_gdf.sindex.query_bulk(poly_gdf['geometry'], predicate="intersects")
        # if len(intersections) != len(poly_gdf):
        #      print('self intersecting polygons')

        
        return poly_gdf

    @timed_function(dt=10)
    def draw_polygons(self):
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
