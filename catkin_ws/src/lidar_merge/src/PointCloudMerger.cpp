#include "PointCloudMerger.h"

namespace lidar_merge {

PointCloudMerger::PointCloudMerger(std::initializer_list<std::string> pointcloud_topics):
    NUM_POINTCLOUDS(pointcloud_topics.size())
{
    for (const auto& topic : pointcloud_topics) {
        pointcloud_subscribers[i] = n.subscribe(topic, QUEUE_SIZE, &PointCloudMerger::receive_pointcloud, this);
    }

    nh.advertise<sensor_msgs::PointCloud2>(lidar_merge::MERGED_POINTCLOUD_TOPIC, QUEUE_SIZE);
}

void PointCloudMerger::receive_pointcloud(const sensor_msgs::PointCloud2& msg) {

}

void PointCloudMerger::merge() {

}

}
