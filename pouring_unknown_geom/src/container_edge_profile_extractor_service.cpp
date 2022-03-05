/*
Copyright (c) <2018>, <Monroe Kennedy III>
All rights reserved
*/

#include <pouring_unknown_geom/basic_ros_include.h>
#include <pouring_unknown_geom/container_edge_profile_extraction.h>
//For GP usage:
#include <libgp/gp.h>
#include <libgp/gp_utils.h>

//based on: http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php
//need conversions: https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/

class ExtractEdgeService
{
public:
  ExtractEdgeService(ros::NodeHandle &nh);


  bool TransformPointCloud(const sensor_msgs::PointCloud2 &inpt_cld, sensor_msgs::PointCloud2 &outpt_cld, std::string &camera_frame); // sensor_msgs::PointCloud2 test; sensor_msgs::PointCloud2Ptr tt (new sensor_msgs::PointCloud2(test)); // for sensor_msgs::PointCloud2ConstPtr

  void FilteredCloudInterestRegion(sensor_msgs::PointCloud2 &filtered_cld, const pouring_unknown_geom::ScanBoxMsg &scan_box);
  void SegmentTable(const sensor_msgs::PointCloud2 &filtered_cld, 
                    sensor_msgs::PointCloud2 &table_pts, 
                    sensor_msgs::PointCloud2 &cup_pts, 
                    pcl::ModelCoefficients::Ptr &table_coef_plane); //This function segments filtered_cld, then returns seperated ind of seperated pt clds and plane eqn 
  std::map<double, sensor_msgs::PointCloud2> ExtractLevelPlanesContainer(const sensor_msgs::PointCloud2 &cup_pt_cld, 
                                                                       const pcl::ModelCoefficients::ConstPtr &table_coef_plane,
                                                                       const float tang_res,
                                                                       double &max_height,
                                                                       std::string camera_frame); //output is heights and point cloud list for levels determined by the specified resolution (container slice resolution)

  std::vector<CylinderData> GenerateCylinders(const std::map<double,sensor_msgs::PointCloud2>  &cyl_height_ref,
                                              std::map<double,sensor_msgs::PointCloud2>  &cyl_slices_filt,
                                              std::map<double,  pcl::ModelCoefficients> &cyl_coefficients_model);  //Need to return radius and height given a) point cloud, b) labeled points (for each cylinder)

  void FitGPData(const std::vector<CylinderData> &cyl_data, 
                 const int &num_pts_return, 
                 std::vector<double> &height_list, 
                 std::vector<double> &radius_list, 
                 double &max_height,
                 std::vector<double> &height_training,
                 std::vector<double> &radius_training);

  bool ProfileExtractor(pouring_unknown_geom::ContainerEdgeProfileSrv::Request &req, pouring_unknown_geom::ContainerEdgeProfileSrv::Response &res);

  bool GenerateFinalProfile(pouring_unknown_geom::ShowFinalEdgeProfileSrv::Request &req, pouring_unknown_geom::ShowFinalEdgeProfileSrv::Response &resp);

  void FilterPtsAboveTable(sensor_msgs::PointCloud2 &filtered_cld, const pcl::ModelCoefficients::Ptr &table_coef_plane, double filt_height_above_table);

  void PlotContainerPoints(const std::map<double,  pcl::ModelCoefficients> cyl_coefficients_model, 
                           const pcl::ModelCoefficients::ConstPtr table_coef_plane, 
                           std::vector<double> height_list, 
                           std::vector<double> radius_list,
                           double max_height,
                           std::string camera_frame,
                           sensor_msgs::PointCloud2 raw_cloud,
                           std::map<double,sensor_msgs::PointCloud2>  &cyl_slices_filt);

  ros::ServiceServer ss;
  ros::ServiceServer show_final_profile;
  ros::ServiceClient gpy_sc;
  ros::Publisher cup_cld_pub; 
  ros::Publisher cup_filtered_cld_pub;
  ros::Publisher relavent_pt_cloud_scan_box_pub;
  ros::Publisher cup_cyl_pt_axis_pub;
  ros::Publisher table_vect_pub;
  ros::Publisher cyl_intersect_pt_pub;

  ros::Publisher lowest_cyl_points_pub;
  ros::Publisher first_cup_slice_pts_pub;


  tf2_ros::StaticTransformBroadcaster static_broadcaster;

private:
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
  tf2_ros::TransformBroadcaster tfbroadcast;
  //For showing the final edge profile
  geometry_msgs::Vector3 global_cup_vect_normed;
  geometry_msgs::Vector3 global_container_table_point;
  geometry_msgs::Vector3 global_cup_vect_perp_normed;
  std::string global_camera_frame;
};

ExtractEdgeService::ExtractEdgeService(ros::NodeHandle &nh) : pnh_(nh)
{
  //organize
  tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
  cup_cld_pub = pnh_.advertise<sensor_msgs::PointCloud2>("container_estimated_cloud", 1, true); //publish estimated points on container surface
  relavent_pt_cloud_scan_box_pub = pnh_.advertise<sensor_msgs::PointCloud2>("scan_box_filtered_pt_cld", 1); //publish estimated points on container surface
  cup_filtered_cld_pub = pnh_.advertise<sensor_msgs::PointCloud2>("filtered_cup_cloud", 1, true); //publish estimated points on container surface
  
  lowest_cyl_points_pub = pnh_.advertise<sensor_msgs::PointCloud2>("lowest_cylinder_pts", 1); //lowest cyl points
  first_cup_slice_pts_pub = pnh_.advertise<sensor_msgs::PointCloud2>("cup_slices_first_pt_set", 1); //lowest cyl points

  cup_cyl_pt_axis_pub = pnh_.advertise<visualization_msgs::MarkerArray>("cyl_pt_axis_array",1);
  cyl_intersect_pt_pub = pnh_.advertise<visualization_msgs::Marker>("cyl_intersect_pt",1);
  table_vect_pub = pnh_.advertise<visualization_msgs::Marker>("table_vector",1);
  ss = pnh_.advertiseService("container_edge_profile", &ExtractEdgeService::ProfileExtractor, this);
  show_final_profile = pnh_.advertiseService("show_final_profile", &ExtractEdgeService::GenerateFinalProfile, this);
  ros::service::waitForService("gp_edge_profile_gen_serv");
  gpy_sc = pnh_.serviceClient<pouring_msgs::GPEdgeProfileGenerator>("gp_edge_profile_gen_serv");
  ROS_INFO("Container Edge Extractor Service Ready");
}


bool ExtractEdgeService::ProfileExtractor(pouring_unknown_geom::ContainerEdgeProfileSrv::Request &req, pouring_unknown_geom::ContainerEdgeProfileSrv::Response &resp)
{
  /*
  req.pt_cld; //point cloud
  req.camera_frame; // camera frame, x toward the object, z upwards
  req.num_pts_return;  //number of points to return 
  req.container_slice_resolution; //in mm, what is the height of a slice to use for contianer resolution (locally fitted with a cylinder)
  req.container_scan_box; //box in specified frame (usually camera_frame) where x points toward target, this box in reference frame specifies region of interest for container/table points to consider
  */
  //1. Given Reference frame (camera_frame), transform point cloud from current frame to this new frame
  sensor_msgs::PointCloud2 pt_cld_oper;
  bool transform_success = TransformPointCloud(req.pt_cld, pt_cld_oper, req.camera_frame);

  if(transform_success){
    //2. Filter points to those within the box of interest [visualize]
    FilteredCloudInterestRegion(pt_cld_oper, req.container_scan_box);
    //3. Use PCL segmentation to extract table vs container points, and retain table plane eqn
    pcl::ModelCoefficients::Ptr table_coef_plane (new pcl::ModelCoefficients);
    sensor_msgs::PointCloud2 table_pt_cld, cup_pt_cld;
    SegmentTable(pt_cld_oper, table_pt_cld, cup_pt_cld, table_coef_plane);
    //Ensure only points above the table are considered
    double filt_height_above_table = 0.005; //meters was 0.01
    FilterPtsAboveTable(cup_pt_cld, table_coef_plane, filt_height_above_table);
    if(cup_pt_cld.data.size() == 0){
      ROS_INFO("No points above the table");
      return false; //no points above the table
    }
    cup_pt_cld.header.frame_id = req.camera_frame;
    cup_filtered_cld_pub.publish(cup_pt_cld);
    pt_cld_oper.header.frame_id = req.camera_frame;
    relavent_pt_cloud_scan_box_pub.publish(pt_cld_oper);
    // std::cout << "number of total points: " << cup_pt_cld.width << std::endl;
    if(cup_pt_cld.width < 30){
      resp.success = false;
      return false;
    }

    //4. For container, extract lowest/highest points in table frame, store h_max (diff btw low/high)
    //5. Using h_max and resolution, divide container points into m = h_max/res regions, with assignment into k'th region from p_{k,i} in [k*res,(k+1)*res]+z_min with k in [0,m-1]. 
    double max_height;
    std::map<double, sensor_msgs::PointCloud2> cup_slices = ExtractLevelPlanesContainer(cup_pt_cld, table_coef_plane, req.container_slice_resolution, max_height, req.camera_frame);
    //6. For each subregion, fit cylinder to the points
    //7. For each cylinder, store the a) radius b) centered height
    
    // std::cout << "frame of cup slices" << cup_slices.begin()->second.header.frame_id << std::endl; // it is pico_camera_link **but not actually correct
    first_cup_slice_pts_pub.publish(cup_slices.begin()->second);



    std::map<double, sensor_msgs::PointCloud2> cup_slices_filtered; //extract the points specifically used to construct the cylinder
    std::map<double,  pcl::ModelCoefficients> cyl_coefficients_model;
    std::vector<CylinderData> cyl_data = GenerateCylinders(cup_slices, cup_slices_filtered, cyl_coefficients_model); 
    //8. Using Gaussian Process with these points as training points, return the specified N points for the function h(r), along with h_max
    std::vector<double> height_list, radius_list, height_training, radius_training;
    FitGPData(cyl_data, req.num_pts_return, height_list, radius_list, max_height,height_training, radius_training);
     //bool for plotting container points
    //Finally populate and return the service response 

    resp.profile.height = height_list;
    resp.profile.radius = radius_list;
    resp.profile.container_height_m = max_height;
    resp.success = true;
    resp.generating_cyl_points.height = height_training;
    resp.generating_cyl_points.radius = radius_training;
    resp.generating_cyl_points.container_height_m = max_height;
    //Finally, plot the points if desired
    if(req.plot_container_points)
    {
      PlotContainerPoints(cyl_coefficients_model, table_coef_plane, height_list, radius_list, max_height, req.camera_frame, req.pt_cld, cup_slices_filtered);
    }

  }
  return true;
}

bool ExtractEdgeService::TransformPointCloud(const sensor_msgs::PointCloud2 &inpt_cld, sensor_msgs::PointCloud2 &outpt_cld, std::string &camera_frame)
{
  //Transform the point cloud into the provided frame
  //1. Obtain tranform btw frames
  geometry_msgs::TransformStamped tf_cam_tran;
  bool tf_success = true; //To determine if tf's were obtained to continue with calculations
  try{
    tf_cam_tran = tfBuffer.lookupTransform(camera_frame, inpt_cld.header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ROS_WARN("Transform not available btw these frames");
    ros::Duration(1.0).sleep();
    tf_success = false;
    return tf_success;
  }
  if(tf_success) //Only proceed if tf's were obtained to perform calculations
  {
   //obtain transform
   Eigen::Affine3d TF_CF = tf2::transformToEigen(tf_cam_tran);
   Eigen::Affine3f TF_CFF = TF_CF.cast<float>();
   pcl_ros::transformPointCloud(TF_CFF.matrix(), inpt_cld, outpt_cld);
   return tf_success;
  }
  // TransformPointCloud( table_frame_name,cup_pt_cld, cup_pt_cld_table_frame, tfListener);
}

void ExtractEdgeService::FilteredCloudInterestRegion(sensor_msgs::PointCloud2 &filtered_cld, const pouring_unknown_geom::ScanBoxMsg &scan_box)
{
 //Given the camera 'interest' frame, (x points toward cup, z upwards), use the scan box in this frame to select only points that are interior to the box in that frame
  //http://wiki.ros.org/pcl/Tutorials & http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough  & http://docs.pointclouds.org/trunk/classpcl_1_1_crop_box.html
  // Container for original & filtered data
  pcl::PCLPointCloud2Ptr cloudPtr(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2Ptr cloudFiltPtr(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(filtered_cld, *cloudPtr);
  //Define the limits of the box
  Eigen::Vector4f min_pt(scan_box.p1.x, scan_box.p1.y, scan_box.p1.z, 1);
  Eigen::Vector4f max_pt(scan_box.p2.x, scan_box.p2.y, scan_box.p2.z, 1);
  //Define the crop box object and filter
  pcl::CropBox<pcl::PCLPointCloud2> crop_box;
  crop_box.setMin(min_pt);
  crop_box.setMax(max_pt);
  crop_box.setInputCloud (cloudPtr);
  crop_box.filter(*cloudFiltPtr);
  // Convert to ROS data type
  pcl_conversions::fromPCL(*cloudFiltPtr, filtered_cld);
}

void ExtractEdgeService::SegmentTable(const sensor_msgs::PointCloud2 &filtered_cld, 
                                      sensor_msgs::PointCloud2 &table_pt_cld, 
                                      sensor_msgs::PointCloud2 &cup_pt_cld, 
                                      pcl::ModelCoefficients::Ptr &table_coef_plane)
{
  //This function segments filtered_cld, then returns seperated ind of seperated pt clds and plane eqn 
  pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(filtered_cld, *cloudPtr);
  
  //Instatiation
  pcl::NormalEstimation<PointT, pcl::Normal> ne; 
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>); 
  pcl::PointIndices::Ptr table_pts (new pcl::PointIndices), cup_pts (new pcl::PointIndices); //table indicies of original point cloud
  
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloudPtr);
  ne.setKSearch (75); //This needs to be tuned, as this determines the scale for who your neighbors are  was 50, then 75
  ne.compute (*cloud_normals);
  
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1); //started with 0.1
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (300); //was 100
  seg.setDistanceThreshold (0.03); //Started with 0.03
  seg.setInputCloud (cloudPtr);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*table_pts, *table_coef_plane); //Send back the table coefficients here

  // Remove the planar inliers, extract the rest
  extract.setInputCloud (cloudPtr);
  extract.setIndices (table_pts);
  extract.setNegative (false);
  //First extract the table plane points
  pcl::PointCloud<PointT>::Ptr cloud_table_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_table_plane); 
  //Then extract the cup points
  pcl::PointCloud<PointT>::Ptr cloud_cup_pts (new pcl::PointCloud<PointT>);
  extract.setNegative (true);
  extract.filter (*cloud_cup_pts);
  //Convert these to types to send back
  pcl::toROSMsg(*cloud_table_plane, table_pt_cld);
  pcl::toROSMsg(*cloud_cup_pts, cup_pt_cld);
} 

void ExtractEdgeService::FilterPtsAboveTable(sensor_msgs::PointCloud2 &filtered_cld, const pcl::ModelCoefficients::Ptr &table_coef_plane, double filt_height_above_table)
{
  /** This function filters points above the table */
  //1. Extract table plane coefficients  (check that z-component is positive, otherwise flip the sign of the vector :: this makes underlying assumption about the halfspace of observation)
    PlaneEqnStruct t_plane;
  t_plane.x = table_coef_plane->values[0];
  t_plane.y = table_coef_plane->values[1];
  t_plane.z = table_coef_plane->values[2];
  t_plane.d = table_coef_plane->values[3];

  if(t_plane.z < 0.0){
    t_plane.x = -t_plane.x; t_plane.y = -t_plane.y; t_plane.z = -t_plane.z; t_plane.d = -t_plane.d; //This flips if the vector is not in the same halfspace as camera frame z
  }

  //2. Loop through pt-cld and for points that are above table, store them in new point cloud
  pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(filtered_cld, *cloudPtr);
  pcl::PointCloud<PointT> newcloud;

  for(int idx = 0; idx < cloudPtr->points.size(); ++idx)
  {
    //a) extract point
    PointT curr_pt = cloudPtr->points[idx]; //http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#a26ff8d864157a2df538216f372405588
    //b) Find plane equation output
    double pln_eqn_outpt = t_plane.x*curr_pt.x + t_plane.y*curr_pt.y + t_plane.z*curr_pt.z + t_plane.d;
    if (pln_eqn_outpt >= filt_height_above_table) //0.01
    {
      newcloud.points.push_back(curr_pt);
    }
  }
  //3. Finally re-assign filtered_cld to the new points that are above the table
  pcl::toROSMsg(newcloud, filtered_cld); /** TODO: add a check if point cloud is populated, break if empty  **/ 
}

std::map<double, sensor_msgs::PointCloud2> ExtractEdgeService::ExtractLevelPlanesContainer(const sensor_msgs::PointCloud2 &cup_pt_cld, 
                                                                                           const pcl::ModelCoefficients::ConstPtr &table_coef_plane,
                                                                                           const float tang_res,
                                                                                           double &max_height,
                                                                                           std::string camera_frame)
{
  //output is heights and point cloud list for levels determined by the specified resolution (container slice resolution)
  //First step is to take slices of the container that align with the table axis (not camera frame, as camera is usually tilted wrt the container)
  /**1. First find the rotated frame that is z aligned with table z-axis and publish static transform */
  Eigen::Vector3f camera_z, table_vect, camera_z_normed, table_vect_normed;
  if(table_coef_plane->values[2] >=0){
    table_vect.x() = table_coef_plane->values[0]; table_vect.y() = table_coef_plane->values[1]; table_vect.z() = table_coef_plane->values[2]; 
  }else{
    table_vect.x() = -table_coef_plane->values[0]; table_vect.y() = -table_coef_plane->values[1]; table_vect.z() = -table_coef_plane->values[2];
  }
  camera_z.x() = 0.0; camera_z.y() = 0.0; camera_z.z() = 1.0;
  camera_z_normed = camera_z.normalized();
  table_vect_normed = table_vect.normalized();
  double rot_angle = acos(camera_z_normed.dot(table_vect_normed)); //Obtain frame rotation angle
  // double rot_angle = 0.707;//float(45)*M_PI/float(180);
  Eigen::Vector3f rot_vector = camera_z_normed.cross(table_vect_normed); //Frame rotation axis
  Eigen::Vector3f rot_vector_normed = rot_vector.normalized();
  // Eigen::Affine3d Rot_transform(Eigen::AngleAxisd(rot_angle,rot_vector_normed)); //Make affine matrix
  std::string table_frame_name = "table_frame";
  Eigen::Quaternionf Rot_quat(Eigen::AngleAxisf(rot_angle,rot_vector_normed));
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = camera_frame;
  static_transformStamped.child_frame_id = table_frame_name;
  static_transformStamped.transform.translation.x = 0.0;
  static_transformStamped.transform.translation.y = 0.0;
  static_transformStamped.transform.translation.z = 0.0;
  static_transformStamped.transform.rotation.x = Rot_quat.x();
  static_transformStamped.transform.rotation.y = Rot_quat.y();
  static_transformStamped.transform.rotation.z = Rot_quat.z();
  static_transformStamped.transform.rotation.w = Rot_quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
  ros::Duration(0.2).sleep(); //give it a moment to load (0.2sec)
  /** 2. Rotate the point cloud into the table aligned frame */
  sensor_msgs::PointCloud2 cup_pt_cld_table_frame;
  TransformPointCloud(cup_pt_cld, cup_pt_cld_table_frame, table_frame_name);
  


  pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>);
  // pcl::fromROSMsg(cup_pt_cld, *cloudPtr);
  pcl::fromROSMsg(cup_pt_cld_table_frame, *cloudPtr);
  //3. For container, extract lowest/highest points in table frame, store h_max (diff btw low/high)
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloudPtr, minPt, maxPt); //These points represent the limits on the point cloud (extremes), these can be used to with the z-values to determine the height of the container
  float h_max = std::abs(maxPt.z - minPt.z);
  max_height = double(h_max);
  //4. Using h_max and resolution, divide container points into m = h_max/res regions, with assignment into k'th region from p_{k,i} in [k*res,(k+1)*res]+z_min with k in [0,m-1]. 
  int m = std::ceil(h_max/tang_res); //Number of sections
  float new_tang_res  = h_max/float(m); //Adjust resolution for even division
  //Given a) h_max b) adjusted resolution, Then using Extract function in a loop, extract all points that lie within regions
  std::map<double, sensor_msgs::PointCloud2> cup_regions_map;
  //Extract the local regions
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloudPtr);
  extract.setNegative (false);
  for(int idx = 0; idx < m; idx++)
  {
    //Estabilish z-bounds for current subset
    double h_local_min = double(idx)*new_tang_res + minPt.z;
    double h_local_max = double(idx+1)*new_tang_res + minPt.z;
    //Filter the points out that are within this range
    pcl::PointCloud<PointT>::Ptr slice_cup_pts (new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;  //Was this handled correctly? correct type?
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (h_local_min, h_local_max);
    pass.filter (*slice_cup_pts);
    //Convert these back to pointcloud2
    sensor_msgs::PointCloud2 slice_cup_pt_cld_table_frame;
    pcl::toROSMsg(*slice_cup_pts, slice_cup_pt_cld_table_frame);
    slice_cup_pt_cld_table_frame.header.frame_id = table_frame_name;
    //Here rotate the point clouds from table frame back into camera frame:
    sensor_msgs::PointCloud2 slice_cup_pt_cld;
    TransformPointCloud(slice_cup_pt_cld_table_frame, slice_cup_pt_cld, camera_frame);
    slice_cup_pt_cld.header.frame_id = camera_frame;
    //Place these and the heights (max) into the map
    double h_dict = 0.5*(h_local_max+h_local_min) - minPt.z; //This is important for curve fitting in last step
    cup_regions_map[h_dict] = slice_cup_pt_cld;
  }
  
  return cup_regions_map;
}

std::vector<CylinderData> ExtractEdgeService::GenerateCylinders(const std::map<double, sensor_msgs::PointCloud2> &cyl_height_ref, 
                                                                std::map<double,sensor_msgs::PointCloud2>  &cyl_slices_filt, 
                                                                std::map<double,  pcl::ModelCoefficients> &cyl_coefficients_model)
{
  //Need to return radius and height given a) point cloud, b) labeled points (for each cylinder)
  std::vector<CylinderData> cyl_data;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; //for fitting cylinders
  //6. For each subregion, fit cylinder to the points

  //For normal segmentation
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::ExtractIndices<PointT> extract;

  for (const auto &myPair : cyl_height_ref)
  {
    
    double key =  myPair.first; //extract the current key which is the height 
    // std::cout << "current height key: " << key << std::endl;   
    sensor_msgs::PointCloud2 current_pt_cld_slice = cyl_height_ref.at(key); //extract the point cloud
  
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    //Since using seg which is segementation from normals, the normals must first be obtained
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    // pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(current_pt_cld_slice, *cloudPtr);
   
    //For finding cylinder, first extract normals
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloudPtr);
    ne.setKSearch (4); // was 50  then 10, 30 decebt  #5 works similarly, but its slower with lower numbers
    ne.compute (*cloud_normals);

    //Fit cylinder to this local cloud
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.61);//.91 was greatthis is angular distance weight 0.3 //latest was 0.3 //was 0.1 , Then 0.3 (decent but not detecting curvature), (0.5 performed well), 0.3 performed well
    seg.setMaxIterations (10000); //was 10000
    seg.setDistanceThreshold (0.1); //distance to the model threshold 0.13// was 0.03  #then 0.01
    seg.setRadiusLimits (0, 0.1); //This will need to be increased for very large containers
    seg.setInputCloud (cloudPtr);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    // Write the cylinder inliers
    extract.setInputCloud (cloudPtr);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ()) 
      std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
      // Store the new map with new point clouds that were used
      sensor_msgs::PointCloud2 cloud_cylinder_ros;
      pcl::toROSMsg(*cloud_cylinder, cloud_cylinder_ros); 
      cloud_cylinder_ros.header.frame_id = current_pt_cld_slice.header.frame_id;
      cyl_slices_filt[key] = cloud_cylinder_ros; // Store the points used to generate the cylinders
      cyl_coefficients_model[key] = *coefficients_cylinder; //Store the cylinder coefficients
      // Store the radius, 
      //7. For each cylinder, store the a) radius b) centered height
      CylinderData curr_cyl_data;
      curr_cyl_data.radius = coefficients_cylinder->values[6];//http://docs.pointclouds.org/1.7.0/group__sample__consensus.html
      // Specifically: The seven coefficients of the cylinder are given by a point on its axis, the axis direction, and a radius, as: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]
      curr_cyl_data.height = key;
      cyl_data.push_back(curr_cyl_data);
    }//end if statement for checking that cylinder fitting is non-empty
  
  }//end for loop through slices
  std::cout << "\n\n";
  for(int idx=0; idx<cyl_data.size(); ++idx){
    // std::cout << "return cylinder data: h:" << cyl_data[idx].height << " r:" << cyl_data[idx].radius << std::endl;
  }
  
  return cyl_data;
  
}//end function

void ExtractEdgeService::FitGPData(const std::vector<CylinderData> &cyl_data, 
                                   const int &num_pts_return, 
                                   std::vector<double> &height_list, 
                                   std::vector<double> &radius_list, 
                                   double &max_height,
                                   std::vector<double> &height_training, 
                                   std::vector<double> &radius_training)
{


  std::vector <double> new_heights = linspace(0.0,max_height,num_pts_return); 
  pouring_msgs::GPEdgeProfileGenerator gp_py_req;

  for(int idx=0;idx<cyl_data.size();++idx){
    height_training.push_back(cyl_data[idx].height);
    radius_training.push_back(cyl_data[idx].radius);
  }
  // std::cout << "hsize" << h_train.size() << "rad_size" << rad_train.size() << std::endl;
  gp_py_req.request.h_train = height_training;
  gp_py_req.request.rad_train = radius_training;
  gp_py_req.request.h_test = new_heights;


  double output_radius;
  if (gpy_sc.call(gp_py_req))
  {
    ROS_INFO("gpy callback successful");
    for (int idx =0; idx < gp_py_req.response.rad_pred.size(); ++idx)
    {
      output_radius = gp_py_req.response.rad_pred[idx];
      radius_list.push_back(output_radius);
      height_list.push_back(new_heights[idx]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service gp_edge_profile_gen_serv");
    //1. For package GP, define training sample size, GP params
    int  n_train_size =  cyl_data.size();
    libgp::GaussianProcess gp(1,"CovSum ( CovSEiso, CovNoise)");
    //initialize hyper param vector
    Eigen::VectorXd params(gp.covf().get_param_dim()); 
    params << -3.0, -1.0,-4.6; //(l**2,a,sigma), for l**2 = 10^(inpt) [sqrt provides expected association, -3.5 -> 0.017m association], using sigma of -4.6 since log10(0.005**2)=-4.6 (assuming error of 0.005m noise)
    //Finally went with -3.0, -1.0,-4.6 (l association of 3cm)
    //-2.5, -1.0,-4.0 Works well
    //-3.2,0.0,-2.0

    //Set params for covariance fnct
    gp.covf().set_loghyper(params);
    //2. Add training data
    // std::cout << "\n\nadding the training data" << std::endl;
    for(int idx = 0; idx < cyl_data.size(); ++idx){
      CylinderData curr_cyl_data = cyl_data[idx];
      //Add pattern
      double h_i[] = {curr_cyl_data.height};
      double r_i = curr_cyl_data.radius;
      // std::cout << "h: " << curr_cyl_data.height << " r: " << curr_cyl_data.radius << std::endl;
      gp.add_pattern(h_i, r_i);   
    }
    std::cout << "\n";
    
    double tss=0, error;
    double out_radius;
    for(int idx = 0; idx < cyl_data.size(); ++idx){
      CylinderData curr_cyl_data = cyl_data[idx];
      //Add pattern
      double h_i[] = {curr_cyl_data.height};
      double r_i = curr_cyl_data.radius;

      out_radius = gp.f(h_i);
      // std::cout << "h: " << curr_cyl_data.height << " r_test: " << out_radius << std::endl;
      error = r_i - out_radius;
      tss += error*error;
    }
    float gp_rmse = std::sqrt(tss/float(cyl_data.size()));
    ROS_INFO("GP rmse: %f",gp_rmse);


    //2. Establish GP components (zero mean) for 1dim function 
    //2a) make test points
    std::vector <double> new_heights = linspace(0.0,max_height,num_pts_return);  
    

    for (int idx =0; idx < num_pts_return; ++idx)
    {
      double h[] = {new_heights[idx]};
      output_radius = gp.f(h);
      height_list.push_back(new_heights[idx]);
      radius_list.push_back(output_radius);
    }
  }

}

void ExtractEdgeService::PlotContainerPoints(const std::map<double,pcl::ModelCoefficients> cyl_coefficients_model, 
                                             const pcl::ModelCoefficients::ConstPtr table_coef_plane, 
                                             std::vector<double> height_list, 
                                             std::vector<double> radius_list,
                                             double max_height,
                                             std::string camera_frame,
                                             sensor_msgs::PointCloud2 raw_cloud, 
                                             std::map<double,sensor_msgs::PointCloud2>  &cyl_slices_filt)
{
  //This function plots the container profile points using the output of the GP to reconstruct the container
  //1. Average all the vectors from the cylinders (axis vectors), after ensuring their postive z-components
  geometry_msgs::Vector3 cup_vect, min_cyl_pt;
  cup_vect.x = 0, cup_vect.y = 0, cup_vect.z = 0; //cylinder vector components
  min_cyl_pt.x = 1e10; min_cyl_pt.y = 1e10; min_cyl_pt.z = 1e10; //Min cylinder points
  double key_h_min = 1e10;
  int N = 0;
  std::cout << "number of cyl coeff: " << cyl_coefficients_model.size() << std::endl;
  visualization_msgs::MarkerArray cyl_marker_array; //For plotting cylinder center/axis

  sensor_msgs::PointCloud2 lowest_cyl_slices_filt;
  pcl::ModelCoefficients lowest_cyl_model_coefficients;

  for (const auto &myPair : cyl_coefficients_model)
  {
    double key = myPair.first;
    //Sum vectors
    if(cyl_coefficients_model.at(key).values[5] >= 0) //Ensure the vectors point up in pico camera frame
    {
    cup_vect.x += cyl_coefficients_model.at(key).values[3]; // point on its axis, the axis direction, and a radius, as: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]
    cup_vect.y += cyl_coefficients_model.at(key).values[4];
    cup_vect.z += cyl_coefficients_model.at(key).values[5];
    }else{
    cup_vect.x -= cyl_coefficients_model.at(key).values[3]; // point on its axis, the axis direction, and a radius, as: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]
    cup_vect.y -= cyl_coefficients_model.at(key).values[4];
    cup_vect.z -= cyl_coefficients_model.at(key).values[5];
    }
    //Check if point is the min point of all reported cylinders
    // std::cout << "cyl z:" << cyl_coefficients_model.at(key).values[2] << std::endl;
    // if(cyl_coefficients_model.at(key).values[2]<min_cyl_pt.z){
    if(key < key_h_min){
      min_cyl_pt.x = cyl_coefficients_model.at(key).values[0];
      min_cyl_pt.y = cyl_coefficients_model.at(key).values[1];
      min_cyl_pt.z = cyl_coefficients_model.at(key).values[2];
      key_h_min = key;
      lowest_cyl_slices_filt = cyl_slices_filt.at(key);
      lowest_cyl_model_coefficients = cyl_coefficients_model.at(key);
    }
    N += 1;
    //Plot cylinder center/axis
    AddCylinderToMarkerArray(cyl_marker_array, cyl_coefficients_model.at(key), camera_frame,N);
  }

  //Plot cyl marker array
  cup_cyl_pt_axis_pub.publish(cyl_marker_array);
  //Average the vectors
  cup_vect.x = (1/float(N))*cup_vect.x; 
  cup_vect.y = (1/float(N))*cup_vect.y;
  cup_vect.z = (1/float(N))*cup_vect.z;
  //2. Use lowest cylinder point and find its distance to the table plane
  PlaneEqnStruct t_plane;
  t_plane.x = table_coef_plane->values[0];
  t_plane.y = table_coef_plane->values[1];
  t_plane.z = table_coef_plane->values[2];
  t_plane.d = table_coef_plane->values[3];
  if(t_plane.z < 0.0){
    t_plane.x = -t_plane.x; t_plane.y = -t_plane.y; t_plane.z = -t_plane.z; t_plane.d = -t_plane.d; //This flips if the vector is not in the same halfspace as camera frame z
  }
  double tbl_pln_distance = t_plane.x*cup_vect.x + t_plane.y*cup_vect.y + t_plane.z*cup_vect.z + t_plane.d;
  geometry_msgs::Vector3 t_plane_vect; t_plane_vect.x = t_plane.x; t_plane_vect.y = t_plane.y; t_plane_vect.z = t_plane.z;

  //3. Find the point on the table for the container (using avg cyl vector, lowest cyl center pt and distance to table plane)
  //Find base point of container
  geometry_msgs::Vector3 container_table_point;
  
  visualization_msgs::Marker intersection_pt_markers; 
  FindCylinderIntersectionPoint(lowest_cyl_slices_filt, lowest_cyl_model_coefficients, *table_coef_plane,  container_table_point, intersection_pt_markers);
  cyl_intersect_pt_pub.publish(intersection_pt_markers);

  //visualize the lowest base cylinder used to locate the container
  lowest_cyl_points_pub.publish(lowest_cyl_slices_filt);
  std::cout << "The frame of the lowest cyc points is " << lowest_cyl_slices_filt.header.frame_id << std::endl;

  //Visualize vector for table (using container table point as intersect)
  visualization_msgs::Marker table_vect_marker;
  PlotTableVector(table_vect_marker, container_table_point, *table_coef_plane, camera_frame);
  table_vect_pub.publish(table_vect_marker);

  //4. Given GP height/radius list, from table pt, avg cyl vector, for every height hp = p + (h)*v establish a perp vector using the 
  //camera reference frame, find the normal component from v (cyl) and normalize: v_perp, then for discrete angles k, rotate the vector  
  //v_perp about v for k angles, and for the radius at that height and the point hp, define the points on the hull of the container and 
  //add this to a point cloud to then publish.

  //4a) First define the perpendicular vector (as the vector cylinder axis is consistent for all points on axis, this needs to be defined only once)
  // geometry_msgs::Vector3 cup_vect_normed = NormalizeVector(cup_vect);
  geometry_msgs::Vector3 cup_vect_normed = NormalizeVector(t_plane_vect); //Try out table plane instead of averaging the cylinder vectors

  geometry_msgs::Vector3 min_cyl_pt_vect_normed = NormalizeVector(min_cyl_pt);
  geometry_msgs::Vector3 cup_vect_perp_normed = PerpVectorNormed(cup_vect, min_cyl_pt);

  //4b) Define angles to rotate
  int angle_steps = 10; //steps btw 0:2pi
  std::vector <double> angles = linspace(0.0,2*M_PI,angle_steps);
  //4c) Define the point cloud to populate
  pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>);
  //4d) Given axis vector, cylinder base point, heights and radii, for each height rotate points on perp vector through defined angles and populate the point cloud
  for(int idx=0; idx<height_list.size(); ++idx)
  {
    double curr_height = height_list[idx];
    double curr_radius = radius_list[idx];
    // Get the current axis point
    geometry_msgs::Vector3 cup_axis_point;
    Eigen::Vector3d cup_axis_point_eig, cup_vect_normed_eig, container_table_point_eig;
    tf::vectorMsgToEigen(cup_vect_normed, cup_vect_normed_eig);
    tf::vectorMsgToEigen(container_table_point, container_table_point_eig);
    cup_axis_point_eig = container_table_point_eig + curr_height*cup_vect_normed_eig; //Find the point along the cup vector above lowest point on the cylinder (min_cyl_pt_eig)
    tf::vectorEigenToMsg(cup_axis_point_eig,cup_axis_point);
    //Add the radius points to the cloud

    AddContainerPoints(angles, cup_vect_normed, cup_vect_perp_normed, cup_axis_point, curr_radius, cloudPtr);
  }
  
  global_cup_vect_normed = cup_vect_normed; //This is for use with coming back to plot the final points
  global_cup_vect_perp_normed = cup_vect_perp_normed; //Global variable for showing final profile
  global_container_table_point = container_table_point; //Global variable

  // std::cout << "this is minpoint right before final application: \n" << min_cyl_pt << std::endl;
  //5. Publish cloud
  //Convert these back to pointcloud2
  sensor_msgs::PointCloud2 container_point_cloud;
  // pcl::PointCloud<PointT> cld_print = *cloudPtr;
  // std::cout << "cld points: " << cld_print << std::endl; //cloutPrt->points.size()
  pcl::toROSMsg(*cloudPtr, container_point_cloud);
  container_point_cloud.header = raw_cloud.header; //Give it same time stamp as raw cloud
  container_point_cloud.header.frame_id = camera_frame; //Adjust the frame-id
  global_camera_frame = camera_frame;
  // std::cout << "Number of points: " << container_point_cloud.width << std::endl;
  cup_cld_pub.publish(container_point_cloud);
}


bool ExtractEdgeService::GenerateFinalProfile(pouring_unknown_geom::ShowFinalEdgeProfileSrv::Request &req, pouring_unknown_geom::ShowFinalEdgeProfileSrv::Response &resp)
{
  // plot the final selected profile
  //private global vars
    // geometry_msgs::Vector3 global_cup_vect_normed;
    // geometry_msgs::Vector3 global_container_table_point;
    // geometry_msgs::Vector3 global_cup_vect_perp_normed;
  // std::string global_camera_frame;
  //Define angles to rotate
  int angle_steps = 10; //steps btw 0:2pi
  std::vector <double> angles = linspace(0.0,2*M_PI,angle_steps);
  pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>);
  //Obtain the 
    for(int idx=0; idx<req.profile.height.size(); ++idx)
  {
    double curr_height = req.profile.height[idx];
    double curr_radius = req.profile.radius[idx];
    // Get the current axis point
    geometry_msgs::Vector3 cup_axis_point;
    Eigen::Vector3d cup_axis_point_eig, cup_vect_normed_eig, container_table_point_eig;
    tf::vectorMsgToEigen(global_cup_vect_normed, cup_vect_normed_eig);
    tf::vectorMsgToEigen(global_container_table_point, container_table_point_eig);
    cup_axis_point_eig = container_table_point_eig + curr_height*cup_vect_normed_eig; //Find the point along the cup vector above lowest point on the cylinder (min_cyl_pt_eig)
    tf::vectorEigenToMsg(cup_axis_point_eig,cup_axis_point);
    //Add the radius points to the cloud
    AddContainerPoints(angles, global_cup_vect_normed, global_cup_vect_perp_normed, cup_axis_point, curr_radius, cloudPtr);
  }
  //5. Publish cloud
  //Convert these back to pointcloud2
  sensor_msgs::PointCloud2 container_point_cloud;
  // pcl::PointCloud<PointT> cld_print = *cloudPtr;
  // std::cout << "cld points: " << cld_print << std::endl; //cloutPrt->points.size()
  pcl::toROSMsg(*cloudPtr, container_point_cloud);
  container_point_cloud.header.stamp = ros::Time::now(); 
  container_point_cloud.header.frame_id = global_camera_frame;
  // std::cout << "Number of points: " << container_point_cloud.width << std::endl;
  cup_cld_pub.publish(container_point_cloud);

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "construct_training_data");
  ros::NodeHandle nh("~");
  ExtractEdgeService  cls_obj(nh); 
  ros::spin();
  return 0;
}



