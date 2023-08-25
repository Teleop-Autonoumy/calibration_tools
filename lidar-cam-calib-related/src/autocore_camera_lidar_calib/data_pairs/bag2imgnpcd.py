# python bag2imgnpcd.py --bag_name 1
import rospy
import rosbag
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv_bridge
from cv2 import imwrite
import numpy as np
import struct
import argparse

def pointcloud2_to_pcd(cloud_msg, filename):
    gen = pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))
    points = np.array([list(pt) for pt in gen])
    
    with open(filename, 'wb') as f:
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
        f.write("DATA binary\n")

        for point in points:
            x, y, z = point[:3]
            packed_point = struct.pack('fff', x, y, z)
            f.write(packed_point)


parser = argparse.ArgumentParser(description='Extract the first frame of a bag file.')
parser.add_argument('--bag_name', type=str, required=True, help='Number for the bag file (e.g. 7 for 7.bag)')
args = parser.parse_args()


# Initialize ROS node
rospy.init_node('bag_first_frame_extractor', anonymous=True)

# Open the bag file
bag_path = '/root/data/{}.bag'.format(args.bag_name)
bag = rosbag.Bag(bag_path, 'r')

# Image topic and PointCloud2 topic
image_topic = '/left_camera/image'
pointcloud_topic = '/rslidar_points_m1'

# Bridge to convert ROS Image messages to OpenCV format
bridge = cv_bridge.CvBridge()

# Flags to check if first frames have been saved
first_image_saved = False
first_pointcloud_saved = False

# Output directory
output_dir = '.'

# Loop through the bag file
for topic, msg, t in bag.read_messages(topics=[image_topic, pointcloud_topic]):
    if topic == image_topic and not first_image_saved:
        # Convert ROS Image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Save image
        imwrite("{}/{}.png".format(output_dir,args.bag_name), cv_image)
        first_image_saved = True

    elif topic == pointcloud_topic and not first_pointcloud_saved:
        # Convert PointCloud2 to PCD and save
        pointcloud2_to_pcd(msg, "{}/{}.pcd".format(output_dir,args.bag_name))
        first_pointcloud_saved = True

    # Exit if both first frames are saved
    if first_image_saved and first_pointcloud_saved:
        break

# Close the bag file
bag.close()

