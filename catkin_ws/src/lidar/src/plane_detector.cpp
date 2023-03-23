#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <audio_feedback/LidarCurb.h>
#include <audio_feedback/LidarObject.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>

ros::Publisher pub_ground;
ros::Publisher pub_ng;
ros::Publisher pub_curb_cloud;
ros::Publisher pub_avg_cloud;
ros::Publisher pub_object_cloud;

ros::Publisher pub_curb;
ros::Publisher pub_objects;

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> seperate_clouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>);

  for(int it : inliers->indices)
  {
    planeCloud->points.push_back(cloud->points[it]);
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
}

void find_objects(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){

  if(input_cloud->size() >  0){
    // pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    // normal_estimator.setSearchMethod(tree);
    // normal_estimator.setInputCloud(input_cloud);
    // normal_estimator.setKSearch(50);
    // normal_estimator.compute(*normals);

    // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    // std::vector <pcl::PointIndices> clusters;
    // reg.setMinClusterSize(50);
    // reg.setMaxClusterSize(200);
    // reg.setSearchMethod(tree);
    // reg.setNumberOfNeighbours(30);
    // reg.setInputCloud(input_cloud);
    // reg.setInputNormals(normals);
    // reg.setSmoothnessThreshold(3.0/180.0*3.1415);
    // reg.setCurvatureThreshold(0.5);
    // reg.extract(clusters);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr color = reg.getColoredCloud();
    // std::cout << " color size " << color->size() << std::endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);
    //std::cout<< "num cluster: " << cluster_indices.size() <<" dude size pls: " << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    audio_feedback::LidarObject objectMsg;
    objectMsg.front = false;
    objectMsg.front_right = false;
    objectMsg.front_left = false;
    objectMsg.back = false;
    for(int i =0; i<cluster_indices.size(); i++){
      //std::cout<<cluster_indices[i].indices.size()<< std::endl;
      if(cluster_indices[i].indices.size() > 500){
        double xObjSum = 0;
        double yObjSum = 0;
        int objPCounter = 0;
        for(int it : cluster_indices[i].indices){
          output_cloud->push_back(input_cloud->points[it]);
          xObjSum += input_cloud->points[it].x;
          yObjSum += input_cloud->points[it].y;
          objPCounter++;
        }
        double objCenterX = xObjSum/objPCounter;
        double objCenterY = yObjSum/objPCounter;
        std::cout<<"obj x: " << objCenterX << " obj Y " << objCenterY << " size " << cluster_indices[i].indices.size() <<std::endl;
        if(objCenterX < 0){
          objectMsg.back = true;
        }else{
          if(objCenterY < -0.25){
            objectMsg.front_right = true;
          }else if(objCenterY > 0.25){
            objectMsg.front_left = true;
          }else{
            objectMsg.front = true;
          }
        }
      }
    }
    pub_objects.publish(objectMsg);
    pcl::PCLPointCloud2 test;
    pcl::toPCLPointCloud2(*output_cloud, test);
    test.header.frame_id = "wheelchair";
    pub_object_cloud.publish(test);
  }
}

void find_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, float dist_thresh, float eps_angle){
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(dist_thresh);
  seg.setInputCloud(input_cloud);
  Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
  seg.setEpsAngle(eps_angle);
  seg.setAxis(axis);
  seg.segment(*inliers, *coefficients);
}

double ground_dist = -1;
void find_curbs (pcl::PCLPointCloud2::Ptr cloud_blob)
{
  double highestZ = -1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr
}

pcl::PointCloud<pcl::PointXYZ>::Ptr sumPointClouds(new pcl::PointCloud<pcl::PointXYZ>);
int counter = 0;
void cloud_cb(const pcl::PCLPointCloud2ConstPtr& cloud_blob){
  // pcl::PCLPointCloud<pcl::PointXYZ> curCloud;
  // pcl::fromROSMsg(*cloud_blob, curCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr distance_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr wheelchair_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud_blob, *converted_cloud);

  pcl::CropBox<pcl::PointXYZ> distanceFilter;
  //0.25 and 0.75
  distanceFilter.setMin(Eigen::Vector4f(-1, -1, -0.5, 1.0));
  distanceFilter.setMax(Eigen::Vector4f(2, 1, 0.5, 1.0));
  distanceFilter.setInputCloud(converted_cloud);
  distanceFilter.filter(*distance_filtered_cloud);

  pcl::PassThrough<pcl::PointXYZ> wheelchair_filter;
  wheelchair_filter.setInputCloud(distance_filtered_cloud);
  wheelchair_filter.setFilterFieldName("x");
  wheelchair_filter.setFilterLimits(-0.3, 0.8);
  wheelchair_filter.setNegative(true);
  wheelchair_filter.filter(*wheelchair_removed_cloud);

  //pcl::PCLPointCloud2 wheelchair_rm_blob;
  //pcl::toPCLPointCloud2(*wheelchair_removed_cloud, wheelchair_rm_blob);
  *sumPointClouds += *wheelchair_removed_cloud;

  if(counter == 5){
    counter = 0;
    pcl::PCLPointCloud2::Ptr sum_cloud(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*sumPointClouds, *sum_cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr avgPtCloud(new pcl::PCLPointCloud2);
    sor.setInputCloud(sum_cloud);
    sor.setLeafSize(0.035f, 0.035f, 0.025f);
    sor.filter(*avgPtCloud);
    find_curbs(avgPtCloud);
    pub_avg_cloud.publish(avgPtCloud);
    sumPointClouds.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }

  counter++;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "plane_detector");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/lidar_merged", 1, cloud_cb);
  pub_ground = nh.advertise<pcl::PCLPointCloud2> ("/plane_cloud", 1);
  pub_avg_cloud = nh.advertise<pcl::PCLPointCloud2> ("/avg_cloud", 1);
  pub_ng = nh.advertise<pcl::PCLPointCloud2> ("/not_ground_cloud", 1);
  pub_curb_cloud = nh.advertise<pcl::PCLPointCloud2> ("/curb_cloud", 1);
  pub_object_cloud = nh.advertise<pcl::PCLPointCloud2> ("/object_cloud", 1);
  pub_curb = nh.advertise<audio_feedback::LidarCurb> ("/lidar/curb_detect", 1);
  pub_objects = nh.advertise<audio_feedback::LidarObject> ("/lidar/object_detect", 1);

  ros::spin ();
}