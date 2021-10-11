//
// Created by yao on 10/8/21.
//



#include <lidar_calibration/data_sync.h>


using namespace std;


int main(int argc,char **argv)
{
    cout<<"Start associating 2 livox data!"<<endl;

    ros::init(argc, argv, "DataAssociation");
    DataSync dataSync;

    // temp test
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//
//    pcl::io::loadPCDFile("/home/yao/Workspace/Livox_automatic_calibration/data/Base_LiDAR_Frames/100023.pcd", *cloud1);
//    pcl::io::loadPCDFile("/home/yao/Workspace/Livox_automatic_calibration/data/Target-LiDAR-Frames/100023.pcd", *cloud2);




    ros::spin();

    return 0;
}
