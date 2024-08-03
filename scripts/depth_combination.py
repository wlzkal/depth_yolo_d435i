#!/usr/bin/env python3

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import math
import tf

# Define image dimensions
height = 720
width = 1280

# Flags and frame identifiers
if_pcl_ready = 0
parent_frame = "camera_color_optical_frame"

def darknet_callback(data):
    global p, if_pcl_ready

    obj_tf = tf.TransformBroadcaster()

    # Check if point cloud data is ready
    if if_pcl_ready:
        bounding_boxes = data.bounding_boxes
        id = 0

        # Iterate through each detected bounding box
        for i in bounding_boxes:
            x, y, z = 0, 0, 0
            valid_num = 0
            length = i.xmax - i.xmin
            height = i.ymax - i.ymin

            # Process points within the bounding box area
            for ix in range(int(i.xmin + length / 4), int(i.xmax - length / 4)):
                for iy in range(int(i.ymin + height / 4), int(i.ymax - height / 4)):
                    index = int((iy - 1) * width + ix)
                    position = p[index]
                    if not math.isnan(position[2]):
                        x += position[0]
                        y += position[1]
                        z += position[2]
                        valid_num += 1

            # Compute average position
            if valid_num:
                x /= valid_num
                y /= valid_num
                z /= valid_num

            # Broadcast the transform for the object
            obj_tf.sendTransform(
                (x, y, z),
                tf.transformations.quaternion_from_euler(0, -math.pi / 2, math.pi / 2),
                rospy.Time.now(),
                i.Class + str(id),
                parent_frame,
            )
            id += 1

def depth_callback(data):
    global p, if_pcl_ready
    # Convert point cloud data to numpy array
    pc = ros_numpy.numpify(data)
    
    # Initialize numpy array for points
    np_points = np.zeros((height * width, 3), dtype=float)  # Use float
    
    # Extract and resize point cloud data
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)
    
    # Create a PointCloud object
    p = np.array(np_points, dtype=float)  # Use float
    if_pcl_ready = 1

def listener():
    rospy.init_node('depth_combination', anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, depth_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Initialize point array
        p = np.zeros((height * width, 3), dtype=float)  # Use float
        listener()
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")

