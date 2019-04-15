#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
import message_filters
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

def get_z(T_cam_world, T_world_pc, K):
    R = T_cam_world[:3,:3]
    t = T_cam_world[:3,3]
    proj_mat = np.dot(K, np.hstack((R, t[:,np.newaxis])))
    xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
    xy_hom = np.dot(proj_mat, xyz_hom.T).T
    z = xy_hom[:, -1]
    z = np.asarray(z).squeeze()
    return z

def extract(point):
    return [point[0], point[1], point[2]]

def callback(scan, image):
    rospy.loginfo("image timestamp: %d ns" % image.header.stamp.to_nsec())
    rospy.loginfo("scan timestamp: %d ns" % scan.header.stamp.to_nsec())
    diff = abs(image.header.stamp.to_nsec() - scan.header.stamp.to_nsec())
    rospy.loginfo("diff: %d ns" % diff)
    img = bridge.imgmsg_to_cv2(image)
    cloud = lp.projectLaser(scan)
    points = pc2.read_points(cloud)
    objPoints = np.array(map(extract, points))
    Z = get_z(q, objPoints, K)
    objPoints = objPoints[Z > 0]
    if lens == 'pinhole':
        img_points, _ = cv2.projectPoints(objPoints, rvec, tvec, K, D)
    elif lens == 'fisheye':
        objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
        img_points, _ = cv2.fisheye.projectPoints(objPoints, rvec, tvec, K, D)
    img_points = np.squeeze(img_points)
    for i in range(len(img_points)):
        try:
            cv2.circle(img, (int(round(img_points[i][0])),int(round(img_points[i][1]))), laser_point_radius, (0,255,0), 1)
        except OverflowError:
            continue
    pub.publish(bridge.cv2_to_imgmsg(img))

rospy.init_node('reprojection')
scan_topic = rospy.get_param("~scan_topic")
image_topic = rospy.get_param("~image_topic")
calib_file = rospy.get_param("~calib_file")
config_file = rospy.get_param("~config_file")
laser_point_radius = rospy.get_param("~laser_point_radius")
time_diff = rospy.get_param("~time_diff")
bridge = CvBridge()
lp = lg.LaserProjection()

with open(calib_file, 'r') as f:
    data = f.read().split()
    qx = float(data[0])
    qy = float(data[1])
    qz = float(data[2])
    qw = float(data[3])
    tx = float(data[4])
    ty = float(data[5])
    tz = float(data[6])
q = Quaternion(qw,qx,qy,qz).transformation_matrix
q[0,3] = tx
q[1,3] = ty
q[2,3] = tz
print("Extrinsic parameter - camera to laser")
print(q)
tvec = q[:3,3]
rot_mat = q[:3,:3]
rvec, _ = cv2.Rodrigues(rot_mat)

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

pub = rospy.Publisher("/reprojection", Image, queue_size=1)
scan_sub = message_filters.Subscriber(scan_topic, LaserScan, queue_size=1)
image_sub = message_filters.Subscriber(image_topic, Image, queue_size=1)
ts = message_filters.ApproximateTimeSynchronizer([scan_sub, image_sub], 10, time_diff)
ts.registerCallback(callback)

rospy.spin()
