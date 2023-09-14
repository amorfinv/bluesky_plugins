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
from plugins.clusters import cluster_algorithms as calgos

from time import time

def init_plugin():

    # Addtional initilisation code
    bs.traf.cluster = Clustering()
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
        
    @timed_function(dt=10)
    def clustering(self):
        
        # First convert the aircraft positions to meters and make observation matrix
        x,y = self.transformer_to_utm.transform(bs.traf.lat, bs.traf.lon)
        features = np.column_stack((x, y))
        
        # normalize the observation matrix
        whitened = whiten(features)

        # do k-means clustering and return the optimal labels
        optimal_labels = calgos.ward(features)

        # polygonize the clusters
        polygons = self.polygonize(optimal_labels, features, n_clusters=20)

    
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
            polygon = Polygon(cluster_points[ hull.vertices])
            polygon = polygon.buffer(buffer_dist*nm)

            polygon_data['label'].append(optimal_label)
            polygon_data['geometry'].append(polygon)

        

            plt.plot(polygon.exterior.xy[0],polygon.exterior.xy[1])

        # create geodataframe of polygons
        poly_gdf = gpd.GeoDataFrame(polygon_data, crs='EPSG:28992')

        # check for potential intersections
        *_,intersections = poly_gdf.sindex.query_bulk(poly_gdf['geometry'], predicate="intersects")

        if len(intersections) != len(poly_gdf):
             print('self intersecting polygons')

        plt.scatter(features[:, 0], features[:, 1], c=optimal_labels)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Optimal Clustering')
        plt.show()
        
        return poly_gdf

