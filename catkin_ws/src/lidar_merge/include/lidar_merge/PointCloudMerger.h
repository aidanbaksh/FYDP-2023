#pragma once

#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

namespace lidar_merge {

inline static constexpr std::string NODE_NAME = 'lidar_merger'
inline static constexpr std::string MERGED_POINTCLOUD_TOPIC = '/lidar_merged'

class PointCloudMerger {
    static constexpr size_t QUEUE_SIZE = 5;

    constexpr size_t NUM_POINTCLOUDS;
public:
    PointCloudMerger(std::initializer_list<std::string>);

    // disallow copy and move
    PointCloudMerger(PointCloudMerger&) = delete;
    PointCloudMerger(PointCloudMerger&&) = delete;

private:
    void receive_pointcloud(const sensor_msgs::PointCloud2&);
    void merge();

    ros::NodeHandle& nh;
    ros::Subscriber tf_static_subscriber;
    std::array<ros::Subscriber, NUM_POINTCLOUDS> pointcloud_subscribers;
    std::array<geometry_msgs::Point
    
    ros::Publisher merged_publisher;
};
  
}
