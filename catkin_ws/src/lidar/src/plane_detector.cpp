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
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>

ros::Publisher pub;
ros::Publisher pub_ng;
ros::Publisher pub_clusters;
ros::Publisher pub_curb_cloud;

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

void formClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud)
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromPCLPointCloud2(*cloud_blob, *object_cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(object_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.08);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(object_cloud);
  ec.extract(cluster_indices);

  uint8_t r = 100, g = 0, b = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Num Objects clustered: " << cluster_indices.size() << std::endl;
  int largest = 0;
  std::vector<pcl::PointIndices>::const_iterator pit = cluster_indices.begin();
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    if(it->indices.size() > largest){
      largest = it->indices.size();
      pit = it;
    }
  }

  for(std::vector<int>::const_iterator it = pit->indices.begin(); it!=pit->indices.end(); ++it){
    output_cloud->points.push_back(object_cloud->points[*it]);
  }
  std::cout<< "test: " << output_cloud->points.size() << std::endl;
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2(*object_cloud, outcloud);
  outcloud.header.frame_id = "wheelchair";
  pub_clusters.publish(outcloud);
  // if(cluster_indices.size() != 0){
  //   std::cout << "Num Points in cluster: " << cluster_indices[0].indices.size() << std::endl;
  //   for(std::vector<int>::const_iterator pit = cluster_indices[0].indices.begin(); pit != cluster_indices[0].indices.end(); ++pit){
  //     output_cloud->points.push_back(object_cloud->points[*pit]);
  //   }
  //   pcl::PCLPointCloud2 outcloud;
  //   pcl::toPCLPointCloud2(*object_cloud, outcloud);
  //   outcloud.header.frame_id = "laser_link";
  //   pub_clusters.publish(outcloud);
  // }
  // for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  // {
  //   if (r != 200){
  //     r += 100;
  //   }else if(g != 200){
  //     g += 100;
  //   }else if(b != 200){
  //     b += 100;
  //   }else{
  //     r = g = b = 0;
  //   }
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //   uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  //   bool first = true;
  //   for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
  //     object_cloud->at(*pit).rgb = *reinterpret_cast<float*>(&rgb);
  //     output_cloud->points.push_back(object_cloud->points[*pit]);
  //     cur_cloud->points.push_back(object_cloud->points[*pit]);
  //   }
  // }
  // pcl::PCLPointCloud2 outcloud;
  // pcl::toPCLPointCloud2(*object_cloud, outcloud);
  // outcloud.header.frame_id = "laser_link";
  // pub_clusters.publish(outcloud);
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

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_blob);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered_blob);

  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  pcl::PassThrough<pcl::PointXYZ> rough_ground_pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rough_ground(new pcl::PointCloud<pcl::PointXYZ>);
  rough_ground_pass.setInputCloud(cloud_filtered);
  rough_ground_pass.setFilterFieldName("z");
  rough_ground_pass.setFilterLimits(-1.0, -0.1);
  rough_ground_pass.filter(*rough_ground);

  pcl::ModelCoefficients::Ptr rough_coefficients (new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr rough_inliers(new pcl::PointIndices());
  find_plane(rough_ground, rough_coefficients, rough_inliers, 0.05, 0.025);
  if(rough_inliers->indices.size() > 0){
    //plane found
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr not_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-abs(rough_coefficients->values[3])-1, -abs(rough_coefficients->values[3])+0.1);
    pass.filter(*ground);
    pass.setNegative(true);
    pass.filter(*not_ground);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    find_plane(ground, coefficients, inliers, 0.03, 0.01);
    if(inliers->indices.size() > 0){
      std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = seperate_clouds(inliers, ground);

      pcl::ModelCoefficients::Ptr curb_coeffs (new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr curb_inliers(new pcl::PointIndices());
      find_plane(not_ground, curb_coeffs, curb_inliers, 0.025, 0.2);

      if(curb_inliers->indices.size() > 0 && abs(abs(curb_coeffs->values[3])-abs(coefficients->values[3])) > 0.05){
        //std::cout<<"curb difference: " << abs(abs(curb_coeffs->values[3])-abs(coefficients->values[3]))<< std::endl;
        double xCurbSum = 0;
        double yCurbSum = 0;
        int curbPCounter = 0;
        for(int it : curb_inliers->indices){
          xCurbSum += not_ground->points[it].x;
          yCurbSum += not_ground->points[it].y;
          curbPCounter++;
        }
        double curbCenterX = xCurbSum/curbPCounter;
        double curbCenterY = yCurbSum/curbPCounter;
        //std::cout << "curb plane center x: " << curbCenterX << " center y: " << curbCenterY << std::endl;
        if(abs(curb_coeffs->values[3]) - abs(coefficients->values[3]) < 0){
          std::cout << "Curb up detected ";
        }else{
          std::cout << "Curb down detected ";
        }
        if(curbCenterX > 0){
          std::cout << "front ";
        }else{
          std::cout << "back ";
        }
        if(curbCenterY > 0.2){
          std::cout << "left";
        }else if(curbCenterY < -0.2){
          std::cout << "right";
        }else{
          std::cout << "center";
        }
        std::cout << std::endl;
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> curbResult = seperate_clouds(curb_inliers, not_ground);
        pcl::PCLPointCloud2 curb_cloud;
        pcl::toPCLPointCloud2(*(curbResult.second), curb_cloud);
        curb_cloud.header.frame_id = "wheelchair";
        pub_curb_cloud.publish(curb_cloud);
      }
      pcl::PCLPointCloud2 ground_cloud;
      pcl::toPCLPointCloud2(*(segResult.second), ground_cloud);
      ground_cloud.header.frame_id = "wheelchair";
      pub.publish(ground_cloud);

    }

      // pcl::PointCloud<pcl::PointXYZ>::Ptr xFiltered(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr yFiltered(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::PassThrough<pcl::PointXYZ> xPass;
      // pcl::PassThrough<pcl::PointXYZ> yPass;
      // xPass.setInputCloud(not_ground);
      // xPass.setFilterFieldName("x");
      // xPass.setFilterLimits(-1.1, 2);
      // xPass.filter(*xFiltered);
      // yPass.setInputCloud(xFiltered);
      // yPass.setFilterFieldName("y");
      // yPass.setFilterLimits(-1, 1);
      // yPass.filter(*yFiltered);

      // pcl::PointCloud<pcl::PointXYZ>::Ptr outliersRemoved(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
      // outlier_filter.setInputCloud(yFiltered);
      // outlier_filter.setMeanK(100);
      // outlier_filter.setStddevMulThresh(0.1);
      // outlier_filter.filter(*outliersRemoved);

      // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      // tree->setInputCloud(outliersRemoved);

      // std::vector<pcl::PointIndices> cluster_indices;
      // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      // ec.setClusterTolerance(0.05);
      // ec.setMinClusterSize(100);
      // ec.setMaxClusterSize(25000);
      // ec.setSearchMethod(tree);
      // ec.setInputCloud(outliersRemoved);
      // ec.extract(cluster_indices);

      // std::cout<<"objects detected: " << cluster_indices.size() << std::endl;
      // for(int i=0; i<cluster_indices.size(); i++){
      //   pcl::PointIndices::Ptr cur_cluster_indices(new pcl::PointIndices());
      //   pcl::CentroidPoint<pcl::PointXYZ> cur_cent_calc;
      //   double x = 0;
      //   double y = 0;
      //   int numCounter = 0;
      //   for(int it : inliers->indices)
      //   {
      //     double cur_x = outliersRemoved->points[it].x;
      //     double cur_y = outliersRemoved->points[it].y;
      //     if(cur_x > -1.1 && cur_x < 2 && cur_y > -1 && cur_y < 1){
      //       x += cur_x;
      //       y += cur_y;
      //       numCounter++;
      //     }
      //     // cur_cent_calc.add(outliersRemoved->points[it]);
      //   }
      //   //pcl::CentroidPoint<pcl::PointXYZ> cur_cent(new pcl::PointXYZ);
      //   // pcl::PointXYZ c1;
      //   // cur_cent_calc.get(c1);
      //   //pcl::computeCentroid(cur_cluster_cloud, cur_cent);
      //   std::cout << "cur centroid x: " << (x/(numCounter)) << " cur cen  y: " << (y/(numCounter))<< " ";
      // }
      // std::cout << std::endl;

      pcl::PCLPointCloud2 not_ground_cloud;
      pcl::toPCLPointCloud2(*not_ground, not_ground_cloud);
      not_ground_cloud.header.frame_id = "wheelchair";
      pub_ng.publish(not_ground_cloud);
  }

  // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = seperate_clouds(inliers, cloud_filtered);
  // pcl::PCLPointCloud2 outcloud;
  // pcl::toPCLPointCloud2(*(segResult.second), outcloud);
  // outcloud.header.frame_id = "wheelchair";
  // pcl::PCLPointCloud2 objectcloud;
  // pcl::toPCLPointCloud2(*(segResult.first), objectcloud);
  // objectcloud.header.frame_id = "wheelchair";
  // pub.publish(outcloud);
  // pub_obst.publish(objectcloud);
  //formClusters((segResult.first));
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "obstacles");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/lidar_merged", 1, cloud_cb);
  pub = nh.advertise<pcl::PCLPointCloud2> ("/plane_cloud", 1);
  pub_ng = nh.advertise<pcl::PCLPointCloud2> ("/not_ground_cloud", 1);
  pub_curb_cloud = nh.advertise<pcl::PCLPointCloud2> ("/curb_cloud", 1);
  pub_clusters = nh.advertise<pcl::PCLPointCloud2> ("/obstacles", 1);

  ros::spin ();
}