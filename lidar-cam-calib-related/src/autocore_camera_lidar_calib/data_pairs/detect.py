#!/usr/bin/env python3

 

import rospy

import cv2

import math

from sensor_msgs.msg import Image, PointCloud2

from cv_bridge import CvBridge

import sensor_msgs.point_cloud2 as pc2

import numpy as np

import struct

from datetime import datetime

import message_filters



 

def pointcloud2_to_pcd(cloud_msg, filename):
    gen = pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))
    points = np.array([list(pt) for pt in gen])
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH {}\n".format(points.shape[0]))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS {}\n".format(points.shape[0]))
        f.write("DATA ascii\n")
        for point in points:
            x, y, z = point
            f.write("{} {} {}\n".format(x, y, z))
 

def callback(img_msg, pc_msg):
    
    global bridge

    try:

        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    except Exception as e:

        print(e)

        return

   

    # Resizing for increased corner detection rate

    height, width = cv_image.shape[:2]

    scale = math.sqrt((width * height) / (640.0 * 480.0))

    if scale > 1.0:

        new_width, new_height = int(width / scale), int(height / scale)

        cv_image_resized = cv2.resize(cv_image, (new_width, new_height))

    else:

        cv_image_resized = cv_image

 

    gray_image = cv2.cvtColor(cv_image_resized, cv2.COLOR_BGR2GRAY)

   

    # New checkerboard size 3x3

    found, corners = cv2.findChessboardCorners(gray_image, (3, 3), None)

   

    if found:

        print("Chessboard corners detected.")

        # Drawing corners on the resized image

        cv2.drawChessboardCorners(cv_image_resized, (3, 3), corners, found)

        cv2.imshow('Corner Detection', cv_image_resized)

        cv2.waitKey(1)

        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        # Save point cloud and image

        pointcloud_filename = './{}.pcd'.format(timestamp_str)

        image_filename = './{}.png'.format(timestamp_str)

        pointcloud2_to_pcd(pc_msg, pointcloud_filename)

        cv2.imwrite(image_filename, cv_image)

        rospy.signal_shutdown("Found corners, shutting down")

 

def pointcloud_callback(pc_msg):

    global latest_pointcloud

    latest_pointcloud = pc_msg

 

if __name__ == "__main__":

    rospy.init_node('corner_detection_and_save', anonymous=True)

    # rospy.set_param('/use_sim_time', True)

    bridge = CvBridge()


 

    image_sub = message_filters.Subscriber('/left_camera/image', Image)

    pointcloud_sub = message_filters.Subscriber('/rslidar_points_m1', PointCloud2)

 

    # Synchronize the topics based on their timestamp

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pointcloud_sub], 10,0.5)

    ts.registerCallback(callback)

 

    rospy.spin()
