//
// Created by yao on 10/11/21.
//

#ifndef SRC_DATA_SYNC_H
#define SRC_DATA_SYNC_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h> // for transformPointCloud
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;

class DataSync
{
public:
    DataSync();
    ~DataSync();

    void LivoxDataCallback(const sensor_msgs::PointCloud2::ConstPtr& msg1,
                            const sensor_msgs::PointCloud2::ConstPtr& msg2);
    void LoadTransformData();
    void TransformAndCombinePointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg1,
                                       const sensor_msgs::PointCloud2::ConstPtr &msg2,
                                       sensor_msgs::PointCloud2::Ptr &output);
    void Publish(const sensor_msgs::PointCloud2::ConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher associated_data_pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub1_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub2_;
    message_filters::Synchronizer<syncPolicy> *sync_;

    Eigen::Matrix4d transform_matrix_;
};


#endif //SRC_DATA_SYNC_H
