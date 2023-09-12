from matplotlib import offsetbox
from bluesky.traffic.turbulence import Turbulence
from bluesky.core.simtime import timed_function
import bluesky as bs
import numpy as np
from bluesky import core
from bluesky import stack
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from shapely.ops import cascaded_union, nearest_points
from shapely.affinity import translate
from bluesky.tools.geo import kwikdist, kwikqdrdist, latlondist, qdrdist
from bluesky.tools.aero import nm, ft, kts
from bluesky.tools.misc import degto180
import time
from scipy.cluster.vq import vq, kmeans, whiten
from pyproj import Transformer
import matplotlib.pyplot as plt

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
        self.transformer_to_utm    = Transformer.from_crs("EPSG:4326", "EPSG:32633")
        self.transformer_to_latlon = Transformer.from_crs("EPSG:32633", "EPSG:4326")
        
    @timed_function(dt=10)
    def clusterfunction(self):
        if bs.traf.ntraf < 100:
            return
        
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
        print("Elbow point is at k=%d" % elbow_index)
        
        # cluster the data with the optimal number of clusters
        optimal_centroids, optimal_distortion = kmeans(whitened, 20)
        optimal_labels, _ = vq(whitened, optimal_centroids)
        
        # plot the data points with different colors for each cluster
        plt.scatter(whitened[:, 0], whitened[:, 1], c=optimal_labels)
        plt.scatter(optimal_centroids[:, 0], optimal_centroids[:, 1], s=100, c='r', marker='x')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Optimal Clustering')
        plt.show()
