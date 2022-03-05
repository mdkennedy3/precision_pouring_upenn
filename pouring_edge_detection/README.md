# Pouring edge detection

This package is responsible for finding the transform to the edge of the pouring container.

# Usage

`rosrun pouring_edge_detection pouring_edge_detection_node`

Once the node is started, and you have your container grasped, request it updates the transforms by calling the service `/edge_detector/update`.  The service will return once the node has successfully updated the transformations.  This may take several seconds because the node has to wait for a new point cloud from the depth camera and the process it.  Once the transform has been updated, the node will continue to publish the same values in the transforms until the next time that they are updated.

# Implementation
The node finds the container by slicing the point cloud into horizontal slices and then using ransac to fit a circle to each subset of points.  The best fit is used as the edge.  It assumes that the container is close to level, that the container is in the gripper, and that only the edge of the container is painted.
