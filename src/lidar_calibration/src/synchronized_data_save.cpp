//
// Created by yao on 10/11/21.
//

#include <lidar_calibration/synchronized_data_save.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>


using namespace std;
string SaveDir="/home/yao/Workspace/extrinsic_calibration_between_two_3d_lidars/Data";

std::string itos(int i)
{
    std::stringstream s;
    s << i;
    return s.str();
}

PointCloudRecord::PointCloudRecord()
{
    cout<<"Start record pointclouds of 2 Lidar"<<endl;
    cloud1_ = new pcl::PointCloud<pcl::PointXYZ>;
    cloud2_ = new pcl::PointCloud<pcl::PointXYZ>;
    count_ = 0;

    lidar_sub1_.subscribe(nh_, "/livox/data1", 1);
    lidar_sub2_.subscribe(nh_, "/livox/data2", 1);
    sync_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), lidar_sub1_, lidar_sub1_);
    sync_->registerCallback(boost::bind(&PointCloudRecord::LivoxDataCallback, this, _1, _2));
}

PointCloudRecord::~PointCloudRecord()
{
    delete cloud1_, cloud2_;
}

void PointCloudRecord::LivoxDataCallback(const sensor_msgs::PointCloud2::ConstPtr& msg1,
                                        const sensor_msgs::PointCloud2::ConstPtr& msg2)
{
    pcl::fromROSMsg(*msg1, *cloud1_);
    pcl::fromROSMsg(*msg2, *cloud2_);

    pcl::io::savePCDFileASCII(SaveDir + "/Lidar1/pointcloud" + itos(count_) + ".pcd", *cloud1_);
    pcl::io::savePCDFileASCII(SaveDir + "/Lidar2/pointcloud" + itos(count_) + ".pcd", *cloud2_);

    count_++;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "DataAssociation");
    PointCloudRecord pointCloudRecord;
    ros::spin();

    return 0;
}