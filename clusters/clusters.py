import numpy as np
from scipy.cluster.vq import vq, kmeans, whiten
from shapely.geometry import Polygon
from scipy.spatial import ConvexHull
import geopandas as gpd
from pyproj import Transformer
import matplotlib.pyplot as plt

import bluesky as bs
from bluesky.core.simtime import timed_function
from bluesky import core

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
        self.kmeansclustering()

    def kmeansclustering(self):
        
        # Identidy 20 clusters
        n_clusters = 20
        # if bs.traf.ntraf < 99:
        #     return
        
        # First step is to convert to meters
        x,y = self.transformer_to_utm.transform(bs.traf.lat, bs.traf.lon)
        
        # create the observation matrix
        features = np.column_stack((x, y))

        # normalize the observation matrix
        whitened = whiten(features)
        
        # calculate the within-cluster sum of squares (WSS) for different values of k
        wss_values = []
        k_values = range(1, 10)
        for k in k_values:
            centroids, distortion = kmeans(whitened, k)
            wss_values.append(distortion)

        # calculate the first and second derivatives of the WSS curve
        first_deriv = np.diff(wss_values)
        second_deriv = np.diff(first_deriv)
        
        # find the elbow point
        elbow_index = np.argmax(second_deriv) + 2
        # print("Elbow point is at k=%d" % elbow_index)
        
        # cluster the data with the optimal number of clusters
        optimal_centroids, optimal_distortion = kmeans(whitened, n_clusters)
        optimal_labels, _ = vq(whitened, optimal_centroids)

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

            polygon_data['label'].append(optimal_label)
            polygon_data['geometry'].append(polygon)

        

            # plt.plot(polygon.exterior.xy[0],polygon.exterior.xy[1])

        # create geodataframe
        poly_gdf = gpd.GeoDataFrame(polygon_data, crs='EPSG:28992')

        # TODO: buffer polygons? and be smart about intersections

        *_,intersections = poly_gdf.sindex.query_bulk(poly_gdf['geometry'], predicate="intersects")

        if len(intersections) != len(poly_gdf):
             print('self intersecting polygons')
        # plt.scatter(features[:, 0], features[:, 1], c=optimal_labels)
        # # plt.scatter(optimal_centroids[:, 0], optimal_centroids[:, 1], s=100, c='r', marker='x')
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.title('Optimal Clustering')
        # plt.show()
