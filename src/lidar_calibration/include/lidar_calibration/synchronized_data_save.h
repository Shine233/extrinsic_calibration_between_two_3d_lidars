//
// Created by yao on 10/11/21.
//

#ifndef SRC_SYNCHRONIZED_DATA_SAVE_H
#define SRC_SYNCHRONIZED_DATA_SAVE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;


class PointCloudRecord
{
public:
    PointCloudRecord();
    ~PointCloudRecord();
    void LivoxDataCallback(const sensor_msgs::PointCloud2::ConstPtr& msg1,
                           const sensor_msgs::PointCloud2::ConstPtr& msg2);

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub1_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub2_;
    message_filters::Synchronizer<syncPolicy> *sync_;
    pcl::PointCloud<pcl::PointXYZI> *cloud1_;
    pcl::PointCloud<pcl::PointXYZI> *cloud2_;
    int count_;
};

#endif //SRC_SYNCHRONIZED_DATA_SAVE_H
