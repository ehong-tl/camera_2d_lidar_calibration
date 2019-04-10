#!/usr/bin/env python

import cv2
import numpy as np
from pyquaternion import Quaternion
import sys
import yaml
import math

def rmse(objp, imgp, lens, K, D, rvec, tvec):
    if lens == 'pinhole':
        predicted, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
        predicted = cv2.undistortPoints(predicted, K, D, P=K)
    elif lens == 'fisheye':
        predicted, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, K, D)
        predicted = cv2.fisheye.undistortPoints(predicted, K, D, P=K)
    predicted = predicted.squeeze()
    imgp = imgp.squeeze()

    pix_serr = []
    for i in range(len(predicted)):
        xp = predicted[i,0]
        yp = predicted[i,1]
        xo = imgp[i,0]
        yo = imgp[i,1]
        pix_serr.append((xp-xo)**2 + (yp-yo)**2)
    ssum = sum(pix_serr)

    return math.sqrt(ssum/len(pix_serr))

if len(sys.argv) == 6:
    config_file = sys.argv[1]
    data_file = sys.argv[2]
    result_file = sys.argv[3]

with open(config_file, 'r') as f:
    f.readline()
    config = yaml.load(f)
    lens = config['lens']
    fx = float(config['fx'])
    fy = float(config['fy'])
    cx = float(config['cx'])
    cy = float(config['cy'])
    k1 = float(config['k1'])
    k2 = float(config['k2'])
    p1 = float(config['p1/k3'])
    p2 = float(config['p2/k4'])

K = np.matrix([[fx, 0.0, cx],
               [0.0, fy, cy],
               [0.0, 0.0, 1.0]])
D = np.array([k1, k2, p1, p2])
print("Camera parameters")
print("Lens = %s" % lens)
print("K =")
print(K)
print("D =")
print(D)

with open(data_file, 'r') as f:
    imgp = []
    objp = []
    for line in f:
        data = line.split()
        objp.append([float(data[0]),float(data[1]),0.0])
        imgp.append([float(data[2]),float(data[3])])

imgp = np.array([imgp],dtype=np.float32)
objp = np.array([objp],dtype=np.float32)

D_0 = np.array([0.0, 0.0, 0.0, 0.0])
retval, rvec, tvec = cv2.solvePnP(objp,imgp,K,D_0,flags = cv2.SOLVEPNP_ITERATIVE)
rmat, jac = cv2.Rodrigues(rvec)
q = Quaternion(matrix=rmat)
print("Transform from camera to laser")
print("T = ")
print(tvec)
print("R = ")
print(rmat)
print("Quaternion = ")
print(q)

print("RMSE in pixel = %f" % rmse(objp, imgp, lens, K, D, rvec, tvec))

with open(result_file, 'w') as f:
    f.write("%f %f %f %f %f %f %f" % (q.x, q.y, q.z, q.w, tvec[0], tvec[1], tvec[2]))

print("Result output format: qx qy qz qw tx ty tz")
