import numpy as np
from scipy.cluster.vq import whiten
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import geopandas as gpd
from pyproj import Transformer
import matplotlib.pyplot as plt

import bluesky as bs
from bluesky.core.simtime import timed_function
from bluesky import core
from bluesky.tools.aero import nm
from bluesky.tools import areafilter as af
from plugins.clusters import cluster_algorithms as calgos

from time import sleep

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
        self.cluster_case = 'intrusions'
    

    @timed_function(dt=10)
    def delete_polygons(self):
        # first delete the areas from bluesky screen
        for areaname, _ in self.polygons_to_draw:            
            af.deleteArea(areaname)
        
        self.polygons_to_draw = []

    @timed_function(dt=10)
    def clustering(self):
        
        features = np.array([])

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

        # polygonize the clusters
        polygons = self.polygonize(optimal_labels, features, n_clusters)

        # code to plot in matplotlib
        self.external_plotting(features, optimal_labels)
        

    def polygonize(self, optimal_labels, features, n_clusters, buffer_dist=bs.settings.asas_pzr*4):
        
        # loop through all of the clusters and create a polygon
        polygon_data = {'label': [], 'geometry': []}

        for optimal_label in range(n_clusters):
            label_mask = optimal_labels == optimal_label
            cluster_points = features[label_mask,:]

            # skip if less than 3 points
            if cluster_points.shape[0] < 3:
                    continue
            hull = ConvexHull(cluster_points)
            polygon = Polygon(cluster_points[hull.vertices])
            polygon = polygon.buffer(buffer_dist*nm)

            polygon_data['label'].append(optimal_label)
            polygon_data['geometry'].append(polygon)


            # save polygons to draw later
            self.polygons_to_draw.append((f'CLUSTER{optimal_label}', polygon))

        # create geodataframe of polygons
        poly_gdf = gpd.GeoDataFrame(polygon_data, crs='EPSG:28992')

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
