#include "lidar_merge/PointCloudMerger.h"

#include <algorithm>
#include <functional>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>


namespace lidar_merge {

PointCloudMerger::PointCloudMerger(ros::NodeHandle& nh):
    nh(nh),
    pointcloud_subscribers(),
    pointcloud_transforms(),
    pointclouds(),
    ready_for_merge(),
    merged_publisher()
{
    // on initialization we have no pointclouds, they cannot be ready to merge
    std::fill(ready_for_merge.begin(), ready_for_merge.end(), false);
}

bool PointCloudMerger::get_lidar_transforms(double timeout) {
    ROS_INFO("Getting lidar transforms...");

    // setup transform listener with 30s buffer
    tf2_ros::Buffer tf_buffer(ros::Duration(30));
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // get transform from each lidar to wheelchair frame
    for (size_t i = 0; i < POINTCLOUD_FRAMES.size(); ++i) {
        try {
            ROS_INFO_STREAM("Looking up transform from " << POINTCLOUD_FRAMES[i]
                << " to " << WHEELCHAIR_FRAME);
            pointcloud_transforms[i] = tf_buffer.lookupTransform(
                WHEELCHAIR_FRAME, POINTCLOUD_FRAMES[i], ros::Time(0), ros::Duration(timeout)
            );
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return false; // failure
        }
    }

    ROS_INFO("Got lidar transforms successfully!");
    return true; // success
}

void PointCloudMerger::start() {
    // create point cloud subscribers
    for (size_t i = 0; i < POINTCLOUD_TOPICS.size(); ++i) {
        pointcloud_subscribers[i] = nh.subscribe<sensor_msgs::PointCloud2>(
            POINTCLOUD_TOPICS[i], QUEUE_SIZE,
            std::bind(&PointCloudMerger::receive_pointcloud, this, std::placeholders::_1, i)
        );
    }

    // create merged pointcloud publisher
    merged_publisher = nh.advertise<sensor_msgs::PointCloud2>(lidar_merge::MERGED_POINTCLOUD_TOPIC, QUEUE_SIZE);

    ROS_INFO("Setup lidar pointclouder subscribers and merge publisher.");
}

void PointCloudMerger::receive_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg, size_t lidar_idx) {
    // extract all points from pointcloud message
    pcl::PointCloud<PointT> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // transform pointcloud to wheelchair frame and store it
    pcl::PointCloud<PointT> cloud_in_wheelchair_frame;
    const geometry_msgs::Transform &transform = pointcloud_transforms[lidar_idx].transform;
    pcl_ros::transformPointCloud(cloud, cloud_in_wheelchair_frame, transform);
    pointclouds[lidar_idx] = std::move(cloud_in_wheelchair_frame);

    // mark this lidar as ready to be merged
    ready_for_merge[lidar_idx] = true;

    // if all lidars are ready to be merged, perform the merge
    if (std::all_of(ready_for_merge.begin(), ready_for_merge.end(), [](bool ready) { return ready; })) {
        merge();
        // after merge is complete, mark all lidars as not ready for merge
        std::fill(ready_for_merge.begin(), ready_for_merge.end(), false);
    }
}

void PointCloudMerger::merge() const {
    pcl::PointCloud<PointT> merged;

    // NOTE: lidar point clouds are already transformed to the wheelchair frame
    for (const auto& lidar_cloud : pointclouds) {
        merged += lidar_cloud;
    }

    // construct ROS msg from merged point clouds and publish
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(merged, msg);
    // frame_id must be appended after converting pcl::PointCloud to sensor_msgs::PointCloud2
    // otherwise the field is overwritten
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = WHEELCHAIR_FRAME;
    merged_publisher.publish(msg);
}

}
