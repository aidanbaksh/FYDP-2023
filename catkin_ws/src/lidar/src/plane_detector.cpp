#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
ros::Publisher pub_obst;
ros::Publisher pub_clusters;

std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> seperate_clouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for(int it : inliers->indices)
  {
    planeCloud->points.push_back(cloud->points[it]);
  }

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
}

void formClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud)
{
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::fromPCLPointCloud2(*cloud_blob, *object_cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(object_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(20);
  ec.setMaxClusterSize(1000);
  ec.setInputCloud(object_cloud);
  ec.extract(cluster_indices);

  uint8_t r = 100, g = 0, b = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Num Objects clustered: " << cluster_indices.size() << std::endl;
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    if (r != 200){
      r += 100;
    }else if(g != 200){
      g += 100;
    }else if(b != 200){
      b += 100;
    }else{
      r = g = b = 0;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    bool first = true;
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
      if(first){
        first = false;
        std::cout << "C x: " << object_cloud->points[*pit].x;
        std::cout << " C y: " << object_cloud->points[*pit].y;
        std::cout << " C z: " << object_cloud->points[*pit].z;
      }
      object_cloud->at(*pit).rgb = *reinterpret_cast<float*>(&rgb);
      output_cloud->points.push_back(object_cloud->points[*pit]);
      cur_cloud->points.push_back(object_cloud->points[*pit]);
    }
  }
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2(*object_cloud, outcloud);
  outcloud.header.frame_id = "laser_link";
  pub_clusters.publish(outcloud);
}

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_blob);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered_blob);

  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.05);
  seg.setInputCloud(cloud_filtered);
  //add back in once mounted and orientation is fixed
  // Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
  // seg.setEpsAngle(0.1);
  // seg.setAxis(axis);

  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0){
    std::cout << "NO PLANES FOUND HELP" << std::endl;
    return;
  }

  std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segResult = seperate_clouds(inliers, cloud_filtered);
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2(*(segResult.second), outcloud);
  outcloud.header.frame_id = "laser_link";
  pcl::PCLPointCloud2 objectcloud;
  pcl::toPCLPointCloud2(*(segResult.first), objectcloud);
  objectcloud.header.frame_id = "laser_link";
  pub.publish(outcloud);
  pub_obst.publish(objectcloud);
  formClusters((segResult.first));
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "obstacles");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/scan_3D_0", 1, cloud_cb);
  pub = nh.advertise<pcl::PCLPointCloud2> ("/plane_cloud", 1);
  pub_obst = nh.advertise<pcl::PCLPointCloud2> ("/obstacles_cloud", 1);
  pub_clusters = nh.advertise<pcl::PCLPointCloud2> ("/obstacles", 1);

  ros::spin ();
}