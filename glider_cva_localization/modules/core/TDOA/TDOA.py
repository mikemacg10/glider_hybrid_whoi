import numpy as np
import matplotlib.pyplot as plt
import itertools
from sklearn.cluster import KMeans

# =============================================================================

def tdoa(positions, targetPose, speedOfSound=1500):
    # positions is a list of 3D vectors
    # targetPose is a 3D vector
    # returns a list of TDOA values
    TOA = np.array([(((position[0] - targetPose[0])**2 + (position[1] - targetPose[1])**2)**0.5) / speedOfSound for position in positions])
    TDOA = np.array([TOA[i] - TOA[j] for i in range(len(TOA)) for j in range(i+1, len(TOA))]) #+ np.random.normal(0, 0.000001, 6)
    return TDOA, TOA

# =============================================================================

def hydrophonePairOrintation(positions, basePose):
    _positions = positions - basePose
    diffs = []
    for i in range(len(_positions)):
        for j in range(i + 1, len(_positions)):
            diffs.append(_positions[j] - _positions[i])
    diffs = np.array(diffs)
    hydrophonePairAngles = np.arctan2(diffs[:, 0], diffs[:, 1])
    hydrophonePairDisplacment = np.linalg.norm(diffs, axis=1)
    return hydrophonePairAngles, hydrophonePairDisplacment

# =============================================================================

def calcAmbigousBearings(TDOA, hydrophonePairAngles, hydrophonePairDisplacment, speedOfSound=1500):
    # Ensure the values are within the range of -1 to 1, or disregard values with large errors (limit is 1.1)    
    values = speedOfSound * TDOA / hydrophonePairDisplacment
    values = np.where(np.abs(values) > 1.1, np.nan, values)
    values = np.clip(values, -1, 1)
    
    acos_values = np.arccos(values)
    ambigousBearings1 = np.rad2deg(circle_minus(acos_values + hydrophonePairAngles))
    ambigousBearings2 = np.rad2deg(circle_minus(2 * np.pi - acos_values + hydrophonePairAngles))
    
    ambigousBearings = np.vstack((ambigousBearings1, ambigousBearings2)).T.flatten()
    
    return ambigousBearings

# =============================================================================

def make_combos(values_in_group, angles):
    combos = np.array(list(itertools.combinations(angles, values_in_group)))
    return combos

# =============================================================================

def min_var(combo_bearings):
    variances = np.var(combo_bearings, axis=1)
    least_var_index = np.nanargmin(variances)
    least_var_combo = combo_bearings[least_var_index]
    mean_bearing = np.mean(least_var_combo, axis=0)
    return mean_bearing

# =============================================================================

def kmeans(angles, numClusters, maxIterations):
    # Reshape the angles array to be a 2D array with one feature
    angles = angles.reshape(-1, 1)

    # Perform k-means clustering
    kmeans = KMeans(n_clusters=numClusters, n_init=maxIterations)

    # Fit the model and predict the clusters
    kmeans.fit(angles)
    clusterCenters = kmeans.cluster_centers_
    
    # Flatten the cluster centers to return a 1D array
    return clusterCenters.flatten()

# =============================================================================

def circle_minus(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi