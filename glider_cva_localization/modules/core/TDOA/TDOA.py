import numpy as np
import matplotlib.pyplot as plt
import itertools
from sklearn.cluster import KMeans

# =============================================================================

def tdoa(positions, targetPose, speedOfSound=1500):
    # positions is a list of 3D vectors
    # targetPose is a 3D vector
    # returns a list of TDOA values
    TOA = ([(((position[0] - targetPose[0])**2 + (position[1] - targetPose[1])**2)**0.5) / speedOfSound for position in positions])
    TDOA = ([TOA[i] - TOA[j] for i in range(len(TOA)) for j in range(i+1, len(TOA))])
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
    weights = np.abs(np.sin(acos_values)**2)
    weights = np.repeat(weights, 2)
    
    ambigousBearings1 = np.rad2deg(circle_minus(acos_values + hydrophonePairAngles))
    ambigousBearings2 = np.rad2deg(circle_minus(2 * np.pi - acos_values + hydrophonePairAngles))
    ambigousBearings = np.vstack((ambigousBearings1, ambigousBearings2)).T.flatten()

    weightDict = {ambigousBearings[i]: weights[i] for i in range(len(ambigousBearings))}
    return ambigousBearings, weightDict

# =============================================================================

def make_combos(values_in_group, angles):
    combos = np.array(list(itertools.combinations(range(len(angles)), values_in_group)))

    combos = np.unique(np.sort(combos, axis=1), axis=0)

    pairs_to_remove = [(0, 1), (2, 3), (4, 5), (6, 7), (8, 9), (10, 11)]

    # Filter out the combos that contain any of the pairs
    filtered_combos = []
    for combo in combos:
        valid = True
        for pair in pairs_to_remove:
            if pair[0] in combo and pair[1] in combo:
                valid = False
                break
        if valid:
            filtered_combos.append(combo)
    combos = np.array(filtered_combos)
    # weightsAoA = np.array([weights[combo].prod() for combo in combos])
    # weightsAoA = weightsAoA / np.sum(weightsAoA)

    combo_bearings = [[angles[i] for i in combo] for combo in combos]
    combo_bearings = np.array(combo_bearings)

    return combo_bearings

# =============================================================================
def min_var(combo_bearings, weightDict, top_n=5):
    # variances = np.var(combo_bearings, axis=1)
    # least_var_index = np.nanargmin(variances)
    # least_var_combo = combo_bearings[least_var_index]
    # mean_bearing = np.mean(least_var_combo, axis=0)
    # return mean_bearing

    variances = np.var(combo_bearings, axis=1)
    top_indices = np.argsort(variances)[:top_n]
    top_combos = combo_bearings[top_indices]
    top_weights = np.array([np.sum([weightDict[bearing] for bearing in combo]) for combo in top_combos])
    top_weights = top_weights / np.sum(top_weights)
    mean_bearings = np.mean(top_combos[np.argmax(top_weights)])
    print(mean_bearings)



    return mean_bearings

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

def getRangeToTarget(targetPose, basePose):
    return np.linalg.norm(targetPose[:2] - basePose[:2])

def circle_minus(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi