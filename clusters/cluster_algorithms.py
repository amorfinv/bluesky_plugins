from sklearn import cluster
import numpy as np
import bluesky as bs

def kmeans(features, n_clusters):
    
    clustering = cluster.KMeans(n_clusters=n_clusters, random_state=0, n_init="auto").fit(features)

    # optimal labels
    optimal_labels = clustering.labels_

    return optimal_labels

def affinity(features):
    
    clustering = cluster.AffinityPropagation(random_state=0).fit(features)

    # optimal labels
    optimal_labels = clustering.labels_

    return optimal_labels

def ward(features):
    
    clustering = cluster.AgglomerativeClustering(n_clusters=None, distance_threshold=3000, linkage='ward').fit(features)

    # optimal labels
    optimal_labels = clustering.labels_

    return optimal_labels

def dbscan(features):
    
    clustering = cluster.DBSCAN(eps=750).fit(features)
 
    # optimal labels
    optimal_labels = clustering.labels_

    return optimal_labels

def hdbscan(features):
    
    clustering = cluster.HDBSCAN(max_cluster_size=int(bs.traf.ntraf*.25)).fit(features)
 
    # optimal labels
    optimal_labels = clustering.labels_

    return optimal_labels

def optics(features):
    
    clustering = cluster.OPTICS(min_samples=0.025).fit(features)
 
    # optimal labels
    optimal_labels = clustering.labels_

    return optimal_labels