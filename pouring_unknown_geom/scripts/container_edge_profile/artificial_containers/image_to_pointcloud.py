#!/usr/bin/env python
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField

from os import listdir
from os.path import isfile, join
import argparse


import point_cloud2

def convert_to_pointcloud(image, focal_length):
    distances = image / 50.0
    fx = focal_length
    fy = focal_length
    cx = image.shape[0] / 2.0
    cy = image.shape[1] / 2.0

    x_coords = np.arange(0, image.shape[0], 1) - cx
    x_coords = x_coords.reshape(1, image.shape[0])
    x_coords = np.repeat(x_coords, image.shape[1], axis=0)
    x_coords = np.ravel(x_coords)

    y_coords = np.arange(0, image.shape[1], 1) - cy
    y_coords = y_coords.reshape(image.shape[1], 1)
    y_coords = np.repeat(y_coords, image.shape[0], axis=1)
    y_coords = np.ravel(y_coords)


    distances = np.ravel(distances)
    
    # Discard all points greater than 5 meters away
    in_view = np.where(distances < 5.0)
    x_coords = x_coords[in_view]
    y_coords = y_coords[in_view]
    distances = distances[in_view]

    x = distances * x_coords / focal_length
    y = -distances * y_coords / focal_length
    z = distances
    
    points = np.rot90(np.array([z, y, x]), axes=(1,0))

    points = map(tuple, points)
    points = np.rec.array(points, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    point_cloud = point_cloud2.array_to_pointcloud2(points)

    return point_cloud
    


    


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert images to bags and publish them')
    parser.add_argument('--image_path')
    parser.add_argument('--num_times_republish', type=int, default=1)
    parser.add_argument('--time_between_publish', type=float, default=1.0)
    parser.add_argument('--frame_id', default="map")

    parser.add_argument('--focal_length', type=float, default=200)
    args = parser.parse_args()

    rospy.init_node("image_to_pointcloud")

    pub = rospy.Publisher("/monstar/points", PointCloud2, queue_size=1)


    image_files = [f for f in listdir(args.image_path) if isfile(join(args.image_path, f))]

    for image in image_files:
        image_mat = cv2.imread(join(args.image_path, image), cv2.IMREAD_UNCHANGED)
        points = convert_to_pointcloud(image_mat, args.focal_length)
        points.header.frame_id = args.frame_id
        for i in range(args.num_times_republish):
            pub.publish(points)
            rospy.loginfo("Publishing: %s", image)
            rospy.sleep(args.time_between_publish)

            if rospy.is_shutdown():
                break
        if rospy.is_shutdown():
            break
