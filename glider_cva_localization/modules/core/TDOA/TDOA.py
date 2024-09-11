import numpy as np
import matplotlib.pyplot as plt
import itertools

def tdoa(positions, targetPose, speedOfSound=1500):
    # positions is a list of 3D vectors
    # targetPose is a 3D vector
    # returns a list of TDOA values
    TOA = []
    for position in positions:
        displacement = (((position[0] - targetPose[0])**2 + (position[1] - targetPose[1])**2)**0.5)
        # print(f"Displacement: {displacement}")
        # print(f"Target Pose: {targetPose}")
        # print(f"Position: {position}")
        TOA.append(displacement/speedOfSound)
    # complete TDOA for all the possinle TOAs
    TDOA = []
    for i in range(len(TOA)):
        for j in range(i+1, len(TOA)):
            TDOA.append(TOA[i] - TOA[j])
    return TDOA

def hydrophonePairOrintation(positions):
    hydrophonePairAngles = []
    hydrophonePairDisplacment = []
    for i in range(4):
        for j in range(i+1, 4):
            angle = np.arctan2(positions[j][0] - positions[i][0], positions[j][1] - positions[i][1])
            hydrophonePairAngles.append(angle)
            hydrophonePairDisplacment.append(((positions[j][0] - positions[i][0])**2 + (positions[j][1] - positions[i][1])**2)**0.5)
        # print(f"Hydrophone pair angles: {np.rad2deg(hydrophonePairAngles)}")
        # print(f"Hydrophone pair displacment: {hydrophonePairDisplacment}")     
    return hydrophonePairAngles, hydrophonePairDisplacment

def calcAmbigousBearings(TDOA, hydrophonePairAngles, hydrophonePairDisplacment, speedOfSound=1500):
    ambigousBearings = []
    for i in range(6):
        value = speedOfSound * TDOA[i] / hydrophonePairDisplacment[i]
        if abs(value) > 1.1:
            value = np.nan
        elif value > 1:
            value = 1
            print("Value is greater than 1")
        elif value < -1:
            value = -1
            print("Value is less than -1")
        ambigousBearings.append(np.rad2deg(circle_minus(np.arccos(value) + hydrophonePairAngles[i])))
        ambigousBearings.append(np.rad2deg(circle_minus(2*np.pi - np.arccos(value) + hydrophonePairAngles[i])))

    return ambigousBearings
def circle_minus(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def make_combos(values_in_group, number_of_values, angles):
    combos = list(itertools.combinations(angles, values_in_group))
    combo_bearings = []
    for combo in combos:
        bearings = []
        for i in range(values_in_group):
            bearings.append(combo[i])
        combo_bearings.append(bearings)
    return combo_bearings

def min_var(combo_bearings):
    least_var = np.nan
    least_var_combo = None

    for bearings in combo_bearings:
        variance = np.var(bearings, axis=0)
        if np.isnan(least_var) or np.any(variance < least_var):
            least_var = variance
            least_var_combo = bearings

    mean_bearing = np.mean(least_var_combo, axis=0)
    return mean_bearing
