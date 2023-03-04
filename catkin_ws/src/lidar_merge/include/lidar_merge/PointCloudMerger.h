#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

namespace lidar_merge {

inline static const std::string NODE_NAME = "lidar_merger";
inline static const std::string MERGED_POINTCLOUD_TOPIC = "/lidar_merged";

class PointCloudMerger {
    // the frame and topic must correspond
    // this is also the 'lidar_index' used in pointclouds and ready_for_merge
    static constexpr size_t NUM_POINTCLOUDS = 3;
    inline static const std::array<std::string, NUM_POINTCLOUDS> POINTCLOUD_TOPICS = {
        "/scan_3D_0",
        "/scan_3D_1",
        "/scan_3D_2",
    };
    inline static const std::array<std::string, NUM_POINTCLOUDS> POINTCLOUD_FRAMES = {
        "back_lidar",
        "front_left_lidar",
        "front_right_lidar",
    };
    inline static const std::string WHEELCHAIR_FRAME = "wheelchair";

    static constexpr size_t QUEUE_SIZE = 5;

    using PointT = pcl::PointXYZ;
    
public:
    PointCloudMerger(ros::NodeHandle&);

    // disallow copy and move
    PointCloudMerger(PointCloudMerger&) = delete;
    PointCloudMerger(PointCloudMerger&&) = delete;

    bool get_lidar_transforms(double timeout = 10);
    void start();

private:
    void receive_pointcloud(const sensor_msgs::PointCloud2::ConstPtr&, size_t);
    void merge() const;

    ros::NodeHandle& nh;

    std::array<ros::Subscriber, NUM_POINTCLOUDS> pointcloud_subscribers;
    std::array<geometry_msgs::TransformStamped, NUM_POINTCLOUDS> pointcloud_transforms;

    std::array<pcl::PointCloud<PointT>, NUM_POINTCLOUDS> pointclouds;
    std::array<bool, NUM_POINTCLOUDS> ready_for_merge;
    
    ros::Publisher merged_publisher;
};
  
}
