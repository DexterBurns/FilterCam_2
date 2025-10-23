#!/usr/bin/env python3

# Import the rclpy library for ROS2 Python nodes
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import Float32
import numpy as np

# Import standard ROS2 message types
from std_msgs.msg import String
from std_msgs.msg import Header

# Clustering libraries
from sklearn.cluster import DBSCAN
from sklearn.cluster import HDBSCAN


class PCDFilter(Node):
    def __init__(self):
        # Initialize the node with a name (used in ROS graph)
        super().__init__('pcdfilter_threshold')

        # Change these to read these values from a service in the future
        self.intensity_high = 101
        self.intensity_low = 99
        self.eps = 0.5
        self.min_samples = 100

        # Declaring parameters 
        self.declare_parameter('low_intensity', 0.0)
        self.declare_parameter('high_intensity', 100.0)
        self.declare_parameter('eps', 0.5)
        self.declare_parameter('min_samples', 100)

        # Create a publisher and subscriber to pointclouds
        self.publisher_ = self.create_publisher(PointCloud2, 'filtered_points', 10)
        self.subscriber_ = self.create_subscription(PointCloud2, 'velodyne_points', self.filter_callback, 10)

        # Create publisher to post distance of the largest detected cluster
        self.publish_cluster_distance = self.create_publisher(Float32, 'cluster_distance', 10)

        self.add_on_set_parameters_callback(self.set_limits)

    # ROS2 Command to set parameters:
    "ros2 param set /threshold_node low_intensity 10.0"
    "ros2 param set /threshold_node high_intensity 10.0"

    def set_limits(self, params):
        for param in params:
            if param.name == 'high_intensity' and param.type_ == Parameter.Type.DOUBLE:
                self.intensity_high = param.value
                self.get_logger().info(f'Updated upper intensity limit to {self.intensity_high}')

            elif param.name == 'low_intensity' and param.type_ == Parameter.Type.DOUBLE:
                self.intensity_low = param.value
                self.get_logger().info(f'Updated lower intensity limit to {self.intensity_low}')

            elif param.name == 'eps' and param.type_ == Parameter.Type.DOUBLE:
                self.eps = param.value
                self.get_logger().info(f'Updated  EPS to {self.eps}')

            elif param.name == 'min_samples' and param.type_ == Parameter.Type.INTEGER:
                self.min_samples = param.value
                self.get_logger().info(f'Updated  min samples to {self.min_samples}')

        return SetParametersResult(successful=True)
    

    # Function to filter the pointcloud data based on intensity.
    # Passing in a ROS2 pointcloud message object
    def filter_callback(self, pointcloud_msg):

        # Loop to parse the pointcloud message that uses list to loop through. 
        points = point_cloud2.read_points(pointcloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans = True)

        # Loop to parse each intensity value and keep only the points we want.
        # This creates a True/False mask that satisfies this condition
        mask = (points['intensity'] > self.intensity_low) & (points['intensity'] < self.intensity_high)

        # Will contain our filtered values
        xyz_tuple = np.empty((len(points), 3))
        for x in range(len(points)):
            if mask[x] == True:
                xyz_tuple[x][0] = (points[x][0])
                xyz_tuple[x][1] = (points[x][1])
                xyz_tuple[x][2] = (points[x][2])

            else:
                xyz_tuple[x][0] = np.nan
                xyz_tuple[x][1] = np.nan
                xyz_tuple[x][2] = np.nan

        # ^^^^^ Everything above is to filter the pointcloud based on intensity ^^^^^

        # Turning back into np to do np logic on it
        np_xyz_tuple = np.array(xyz_tuple)

        # Getting the valid points with no NaN values and the mask
        valid_points_no_NaNs, valid_points_no_NaNs_mask, valid_points_no_NaNs_truth_count = FilterNaNPoints(np_xyz_tuple)

        # Clustered points. Should be in xyz coordinates
        clustered_points = DBSCAN(eps = self.eps, min_samples = self.min_samples, algorithm = 'kd_tree').fit(valid_points_no_NaNs)

        # Assign each value from the DBSCAN back to the valid points with no NaN's. An array corresponding each valid cleaned point to a cluster
        clustered_points.labels_ = np.reshape(clustered_points.labels_, (-1,1))
        valid_points_and_cluster_map = np.hstack( (valid_points_no_NaNs,clustered_points.labels_) )

        # Call function to map the cluster labels back to the original pointcloud 
        array_with_pointcloud_mapped_to_each_cluster = MapLabelsToPoints(valid_points_and_cluster_map, np_xyz_tuple, valid_points_no_NaNs_mask)
           
        # Get the label of the larget occuring cluster 
        largest_cluster_value = GetLargestCluster(clustered_points.labels_)

        # if we get this value, that means there are NO clusters in the pointcloud currently.
        # We will just publish all NaN values to the publisher
        if largest_cluster_value == 999:
            # Fill an empty numpy array with NaN's the same size as the original pcloud
            no_points_tuple_np = np.full(np_xyz_tuple.shape, np.nan)

            # Converting the empty numpy array back to a regular tuple
            empty_points_tuple = [tuple(pt) for pt in no_points_tuple_np]

            # Convert back to pointcloud2 message
            header = Header()
            header.stamp = pointcloud_msg.header.stamp
            header.frame_id = pointcloud_msg.header.frame_id

            # Creating pointcloud with just the xyz coordinates. Creating float message
            filtered_msg = point_cloud2.create_cloud_xyz32(header, empty_points_tuple)
            distance_msg = Float32()

            distance = 0
            distance_msg.data = float(distance)

            # Publishing to our publisher, the empty values
            self.publisher_.publish(filtered_msg)
            self.publish_cluster_distance.publish(distance_msg)
            
        else:
            # Function to get only the largest cluster points
            points_of_largest_cluster = GetPointsofTheLargestCluster(largest_cluster_value, array_with_pointcloud_mapped_to_each_cluster)

            distance = GetLargestClusterDistance(points_of_largest_cluster)

            # Converting the numpy array back to a regular tuple
            points_tuple = [tuple(pt) for pt in points_of_largest_cluster]

            # Convert back to pointcloud2 message
            header = Header()
            header.stamp = pointcloud_msg.header.stamp
            header.frame_id = pointcloud_msg.header.frame_id

            # Creating pointcloud with just the xyz coordinates. Creating float message
            filtered_msg = point_cloud2.create_cloud_xyz32(header, points_tuple)
            distance_msg = Float32()

            distance_msg.data = float(distance)

            # Publishing to our publisher
            self.publisher_.publish(filtered_msg)
            self.publish_cluster_distance.publish(distance_msg)

# Function to map each validpoint-clusterID pair back to the original pointcloud
# Output is that each original pointcloud point shouuld have a cluster ID (or nan if not in a cluster)
def MapLabelsToPoints(vpoint_cluster_pair, original_pointcloud, vpoint_cluster_pair_mask):

    # clustered points labels | valid points no nans = these match up
    # valid points no nan mask gets us from trhe clustered points labels back to the original points 
    # Everything below here shoud probably go into a function on its own
    # Now use the NaNs mask to figure out where each point and what its cluster is.
    counter = 0
    id_holder = np.empty( (len(original_pointcloud), 1))# array we will use to stack the clusterID's onto our original pointcloud 
    for x in range(len(original_pointcloud)):

        # If the position in the mask is true. I.e the point has an associated cluster ID
        if vpoint_cluster_pair_mask[x] == True:

            # Get the cluster ID of that position from the vpoint cluster pair 
            id_holder[x, 0] = vpoint_cluster_pair[counter, 3]
            counter = counter + 1
        
        else: #If the point has no associated cluster ID. set to NaN
             id_holder[x, 0] = np.nan

    # Stack the id_holder onto the original pointcloud
    pointcloud_with_cluster_ids = np.hstack( (original_pointcloud, id_holder) )

    return pointcloud_with_cluster_ids

    
# Function to filter an array and remove the NaN points within it
def FilterNaNPoints(nan_array):

    valid_mask = ~np.isnan(nan_array).any(axis=1)

    valid_points = nan_array[valid_mask]

    truth_count = np.count_nonzero(valid_mask)

    return valid_points, valid_mask, truth_count

# Function to get the euclidian distance of the
def GetLargestClusterDistance(largest_cluster_points):

    cleaned_cluster, _, _= FilterNaNPoints(largest_cluster_points)

    distance = np.linalg.norm(cleaned_cluster, axis=1)

    distance = np.mean(distance)

    return distance

def RemoveNoiseFromCluster(input_cluster):

    valid_mask = [False] * len(input_cluster)
    valid_mask_truth_count = 0

    # This loop to create a mask of the valid values from the input cluster
    for x in range(len(input_cluster)):
        if input_cluster[x] < 0:
            valid_mask[x] = False

        else:
            valid_mask[x] = True
            valid_mask_truth_count = valid_mask_truth_count + 1

    # Create an array the size of the truth count so we can return only the valid values
    valid_clusters = [0] * valid_mask_truth_count

    count = 0
    for x in range(len(valid_mask)):
        if valid_mask[x] == True:
            valid_clusters[count] = input_cluster[x]
            count =  count + 1

        else:
            'Do nothing'

    return valid_clusters, valid_mask, count


# This function will return the label of the largest cluster. A single value
def GetLargestCluster(labels):

    # Get the largest object in the pointcloud by looking at its cluster size
    # First remove the -1's out of the labels array. Bincount doesnt like it 
    # This should give us a mask where True for a cluster point, False for noise
    _, labels_mask, valid_labels_count = RemoveNoiseFromCluster(labels)

    # Define an array, that is the length of the truth values
    labels_of_valid_clusters = np.empty(valid_labels_count)

    # Populate that array with only the unique truth values
    counter = 0
    for x in range(len(labels_mask)):
        if labels_mask[x] == True: # If the mask position is true 
            labels_of_valid_clusters[counter] = labels[x] # Add it to the true labels 
            counter = counter + 1

        else:
            "Do nothing"

    # Converting array to in64 for bincount function
    labels_of_valid_clusters = labels_of_valid_clusters.astype(np.int64)

    # If there are no valid points, return the largest int64 number and break from the function
    if labels_of_valid_clusters.size == 0:
        biggest_occurring_cluster = 999
        return biggest_occurring_cluster
    
    else:
        # Array to get the unique numbers in the labels_of_valid_clusters
        unique_array = np.bincount((labels_of_valid_clusters.flatten()))
        biggest_occurring_cluster = np.argmax(unique_array)

    return biggest_occurring_cluster

def GetPointsofTheLargestCluster(largest_cluster_label_id, pointcloud_with_cluster_ids):

    # Create mask that filters out only points with the label of the largesrt cluster
    # Only where the cluster label is equal to the cluster label in the function
    mask = (pointcloud_with_cluster_ids[:, 3] == largest_cluster_label_id)

    # Cluster points is only where the cluster id matches with the values
    cluster_points = pointcloud_with_cluster_ids[mask, :3] 

    return cluster_points


        
def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    node = PCDFilter()

    # Keep the node alive and responsive to callbacks
    rclpy.spin(node)

    # Cleanup after shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
