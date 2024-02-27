#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/centroid.h>

class LaserScanToPointCloudConverter {
public:
    LaserScanToPointCloudConverter() {
        // Initialize the ROS node
        nh = ros::NodeHandle("~");

        // Subscribe to the LaserScan topic
        laserScanSub = nh.subscribe("/scan", 1, &LaserScanToPointCloudConverter::laserScanCallback, this);

        // Advertise the PointCloud2 topic
        pointCloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/filtered_scan", 1);

        // Advertise the clustered PointCloud2 topic
        clusteredPointCloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/clustered_point_cloud", 1);

        // Advertise the Marker topic
        markerPub = nh.advertise<visualization_msgs::Marker>("/clustered_marker_topic", 1);

    }

    // Callback for the LaserScan topic
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanMsg) {
        // Convert sensor_msgs/LaserScan to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Populate the PCL point cloud with laser scan data within the desired angle range
        for (size_t i = 0; i < laserScanMsg->ranges.size(); ++i) {
            float range = laserScanMsg->ranges[i];
            float angle = laserScanMsg->angle_min + i * laserScanMsg->angle_increment;

            // Only include points within the front 30 degrees
            // if (angle >= 2.35 && angle <= 3.92) {  // Angle range: -30 to 30 degrees
                if (angle >= 2.35 || angle <= -2.35) { 
                pcl::PointXYZ point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.z = 0.0;  // Assuming 2D laser scan data

                cloud->push_back(point);
            }
        }
        
        // Apply Passthrough filter to the point cloud
        applyPassthroughFilter(cloud);

        // Publish the filtered PCL point cloud on a ROS topic
        publishPointCloud(cloud);

        // Perform clustering on the filtered point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        performClustering(cloud, clusteredCloud);

        // Publish the clustered PCL point cloud on a ROS topic
        publishClusteredPointCloud(clusteredCloud);

        // Extract a point from the clustered cloud and publish it as a marker
        // extractAndPublishMarker(clusteredCloud);
        extractAndPublishMarker(clusteredCloud, laserScanMsg);
    }

    // Apply Passthrough filter to the point cloud
    void applyPassthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Create the Passthrough filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(inputCloud);
        pass.setFilterFieldName("x");  // Assuming x is the horizontal axis
        pass.setFilterLimits(-0.8, 0);  // Adjust as needed
        pass.filter(*filteredCloud);

        // Optionally, you can apply additional filtering on other axes if necessary
        // pass.setFilterFieldName("y");
        // pass.setFilterLimits(-5.0, 5.0);
        // pass.filter(*filteredCloud);

        *inputCloud = *filteredCloud;  // Update the input cloud with the filtered result
    }

    // Perform clustering on the point cloud
    void performClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& clusteredCloud) 
    {
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(inputCloud);

        // Create a vector to store the cluster indices
        std::vector<pcl::PointIndices> clusterIndices;

        // Create the EuclideanClusterExtraction object
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.04);  // Set the distance threshold for clustering  0.04
        ec.setMinClusterSize(25);     // Set the minimum cluster size 25
        ec.setMaxClusterSize(60);    // Set the maximum cluster size 40
        ec.setSearchMethod(tree);
        ec.setInputCloud(inputCloud);
        ec.extract(clusterIndices);

        // Extract the clusters
        clusteredCloud->clear();
        for (const auto& indices : clusterIndices) {
            for (const auto& index : indices.indices) {
                clusteredCloud->push_back((*inputCloud)[index]);
            }
        }
    }

    // Extract a point from the clustered cloud and publish it as a marker
    // void extractAndPublishMarker(const pcl::PointCloud<pcl::PointXYZ>::Ptr& clusteredCloud) {
    //     if (!clusteredCloud->empty()) {
    //         // Choose the first point from the clustered cloud
    //         pcl::PointXYZ markerPoint = clusteredCloud->points[0];

    //         // Publish the point as a marker
    //         publishMarker(markerPoint);
    //     }
    // }
    // Extract a point from the clustered cloud and publish it as a marker with angle information
    // Extract the center point from the clustered cloud and publish it as a marker with angle information
    void extractAndPublishMarker(const pcl::PointCloud<pcl::PointXYZ>::Ptr& clusteredCloud,
                             const sensor_msgs::LaserScan::ConstPtr& laserScanMsg) {
    if (!clusteredCloud->empty()) {
        // Compute the centroid of the clustered cloud
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusteredCloud, centroid);

        // Create a pcl::PointXYZ point representing the centroid
        pcl::PointXYZ centerPoint;
        centerPoint.x = centroid[0];
        centerPoint.y = centroid[1];
        centerPoint.z = centroid[2];

        // Calculate the index of the chosen point in the clustered cloud (for angle information)
        size_t index = clusteredCloud->points.size() / 2;
        // Calculate the angle of the point using its index in the original LaserScan message
        float angle = laserScanMsg->angle_min + index * laserScanMsg->angle_increment;

        // Publish the center point of the cluster as a marker with angle information
        publishMarker(centerPoint, angle);
        // Convert radians to degrees
        float angleInDegrees = angle * 180.0 / M_PI;

        // Print the angle in degrees to the terminal
        ROS_INFO("Extracted Angle: %f degrees", angleInDegrees);
    }
    }



    // Publish the clustered PCL point cloud on a ROS topic
    void publishClusteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(*cloud, cloudMsg);
        cloudMsg.header.stamp = ros::Time::now();
        cloudMsg.header.frame_id = "laser";  // Provide the appropriate frame ID

        clusteredPointCloudPub.publish(cloudMsg);
    }

    // Publish the PCL point cloud on a ROS topic
    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(*cloud, cloudMsg);
        cloudMsg.header.stamp = ros::Time::now();
        cloudMsg.header.frame_id = "laser";  // Provide the appropriate frame ID

        pointCloudPub.publish(cloudMsg);
    }

    
     // Publish a marker at the specified point
    // void publishMarker(const pcl::PointXYZ& point) {
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "laser";  // Provide the appropriate frame ID
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "clustered_marker";
    //     marker.id = 0;
    //     marker.type = visualization_msgs::Marker::SPHERE;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = point.x;
    //     marker.pose.position.y = point.y;
    //     marker.pose.position.z = point.z;
    //     marker.pose.orientation.w = 1.0;
    //     marker.scale.x = 0.05;
    //     marker.scale.y = 0.05;
    //     marker.scale.z = 0.05;
    //     marker.color.r = 1.0;
    //     marker.color.g = 0.0;
    //     marker.color.b = 0.0;
    //     marker.color.a = 1.0;

    //     // Publish the marker
    //     markerPub.publish(marker);
    // }

    // Publish a marker at the specified point with angle information
void publishMarker(const pcl::PointXYZ& point, float angle) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";  // Provide the appropriate frame ID
    marker.header.stamp = ros::Time::now();
    marker.ns = "clustered_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Include angle information in the marker
    marker.text = "Angle: " + std::to_string(angle);

    // Publish the marker
    markerPub.publish(marker);
}



private:
    ros::NodeHandle nh;
    ros::Subscriber laserScanSub;
    ros::Publisher pointCloudPub;
    ros::Publisher clusteredPointCloudPub;
    ros::Publisher markerPub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_to_point_cloud_converter");
    LaserScanToPointCloudConverter converter;
    ros::spin();
    return 0;
}
