#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>

#include <pcl/visualization/cloud_viewer.h>

#include <eigen3/Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <std_srvs/Empty.h>

#include <tf/transform_broadcaster.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;


class EdgeDetector {
  public:
    EdgeDetector();
  private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool updateContainerPos(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    ros::ServiceServer service;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher marker_pub_;
    tf::TransformBroadcaster tf_broadcaster;

    tf::Transform center_transform;
    tf::Transform edge_transform;

    std::string base_frame_name;
    std::string center_frame_name;
    std::string edge_frame_name;

    bool updating;
};

EdgeDetector::EdgeDetector() {
    ros::NodeHandle nh;
    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/roboception/stereo/points2", 4, boost::bind(&EdgeDetector::pointcloudCallback, this, _1));
    marker_pub_ = nh.advertise<visualization_msgs::Marker>( "/pouring_edge", 0 );

    service = nh.advertiseService("edge_detector/update", &EdgeDetector::updateContainerPos, this);


    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("base_frame_name", base_frame_name, "roboception_camera");
    private_nh.param<std::string>("center_frame_name", center_frame_name, "pouring_edge_center");
    private_nh.param<std::string>("edge_frame_name", edge_frame_name, "pouring_edge");

    updating = true;

    center_transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    center_transform.setRotation(q);
    edge_transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    edge_transform.setRotation(q);



    ros::Rate rate(10);

    while (ros::ok()) {
        tf_broadcaster.sendTransform(tf::StampedTransform(center_transform, ros::Time::now(), base_frame_name, center_frame_name));
        tf_broadcaster.sendTransform(tf::StampedTransform(edge_transform, ros::Time::now(), base_frame_name, edge_frame_name));
        ros::spinOnce();
        rate.sleep();
    }
}


/*
 * This defines the service call for updating the container pos. It will return once the pos has been updated
 */
bool EdgeDetector::updateContainerPos(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    updating = true;

    ros::Rate r(1);

    while (ros::ok() && updating) {
        ros::spinOnce();
        r.sleep();
    }

    return true;
}


void EdgeDetector::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (! updating) {
        return;
    }

    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    PointCloud::Ptr cloud (new PointCloud);

    pcl::fromROSMsg(*msg, *cloud);

    // printf("PCL: width=%d height=%d\n", cloud->width, cloud->height);


    // printf("Cloud size: %d\n", (int)(cloud->size()));


    int inlier_threshold = 200;
    double slice_height = 0.025;

    int best_num_inliers = 0;
    Eigen::VectorXf best_model_coeffs;


    for (double start_height = 0.1; start_height > -0.1; start_height -= slice_height / 2) {
        pcl::IndicesPtr valid_indices (new std::vector<int>);
        for(int i = 0; i<cloud->size(); i++) {
            if (!std::isnan(cloud->points[i].x)) {
                if (cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z < 0.2*0.2) {
                    if (cloud->points[i].y > start_height && cloud->points[i].y < start_height + slice_height) {
                        valid_indices->push_back(i);
                    }
                }
            }
        }

        PointCloud::Ptr cloud_out (new PointCloud);
        pcl::ExtractIndices<Point> eifilter (true); // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud (cloud);
        eifilter.setIndices (valid_indices);
        eifilter.filter (*cloud_out);

        // pcl::visualization::CloudViewer viewer ("Slice");
        // viewer.showCloud (cloud_out);
        // while (!viewer.wasStopped ())
        // {
        // }

        pcl::SampleConsensusModelCircle3D<Point>::Ptr
            model_p (new pcl::SampleConsensusModelCircle3D<Point> (cloud_out));

        pcl::IndicesPtr inliers (new std::vector<int>);
        pcl::RandomSampleConsensus<Point> ransac (model_p);
        ransac.setDistanceThreshold (.001);
        ransac.computeModel();

        
        Eigen::VectorXf model_coeffs;
        ransac.getModelCoefficients(model_coeffs); 

        for (int i = 0; i<model_coeffs.size(); i++) {
            std::cout << model_coeffs[i] << " ";
        }
        std::cout << "\n";
        ransac.getInliers(*inliers);

        printf("Num inliers: %d / %d\n", (int)(inliers->size()), (int)(cloud->points.size()));

        
        if (inliers->size() > best_num_inliers) {
            best_num_inliers = (int)(inliers->size());
            best_model_coeffs = model_coeffs;
        }


        // PointCloud::Ptr filtered_cloud (new PointCloud);
        // pcl::ExtractIndices<Point> eifilter2 (true); // Initializing with true will allow us to extract the removed indices
        // eifilter2.setInputCloud (cloud_out);
        // eifilter2.setIndices (inliers);
        // eifilter2.filter (*filtered_cloud);

        // pcl::visualization::CloudViewer viewer2 ("Inliers");
        // viewer2.showCloud (filtered_cloud);
        // while (!viewer2.wasStopped ())
        // {
        // }
    }

    visualization_msgs::Marker marker;
    marker.header = msg->header;
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 2*best_model_coeffs[3];
    marker.scale.y = slice_height / 5;  
    marker.scale.z = 2*best_model_coeffs[3];
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.pose.position.x = best_model_coeffs[0];
    marker.pose.position.y = best_model_coeffs[1];
    marker.pose.position.z = best_model_coeffs[2];


    marker_pub_.publish( marker );
    
    center_transform.setOrigin( tf::Vector3(best_model_coeffs[0], best_model_coeffs[1], best_model_coeffs[2]) );
    edge_transform.setOrigin( tf::Vector3(best_model_coeffs[0] + best_model_coeffs[3], best_model_coeffs[1], best_model_coeffs[2]) );
    updating = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "EdgeDetector");
    EdgeDetector e;
}
