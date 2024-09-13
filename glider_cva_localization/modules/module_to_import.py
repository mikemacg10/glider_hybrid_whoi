# import numpy as np

# def tdoa(positions, targetPose, speedOfSound=1500):
#     # positions is a list of 3D vectors
#     # targetPose is a 3D vector
#     # returns a list of TDOA values
#     TOA = []
#     for position in positions:
#         TOA.append(np.linalg.norm(position - targetPose)/speedOfSound)
#     # complete TDOA for all the possinle TOAs
#     TDOA = []
#     for i in range(len(TOA)):
#         for j in range(i+1, len(TOA)):
#             TDOA.append(TOA[i] - TOA[j])
#             print(f"Hydrophone {i} - Hydrophone {j}: {TOA[i] - TOA[j]}")
#     return TDOA

from modules.core.TDOA import tdoa

def localization(positions, targetPose, speedOfSound=1500):
    return tdoa(positions, targetPose, speedOfSound)