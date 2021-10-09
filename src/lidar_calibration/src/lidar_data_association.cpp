//
// Created by yao on 10/8/21.
//

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


using namespace std;

void LivoxDataCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc,char **argv)
{
    cout<<"Start associating 2 livox data!"<<endl;

    ros::init(argc, argv, "DataAssociation");
    ros::NodeHandle nh;
    ros::Subscriber livox_data1_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/data1", 1, LivoxDataCallback);
    ros::Subscriber livox_data2_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/data2", 1, LivoxDataCallback);
    ros::Publisher associated_data_pub = nh.advertise<sensor_msgs::PointCloud2> ("/livox/associated",1);


    // temp test
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("/home/yao/Workspace/Livox_automatic_calibration/data/Base_LiDAR_Frames/100023.pcd", *cloud1);
    pcl::io::loadPCDFile("/home/yao/Workspace/Livox_automatic_calibration/data/Target-LiDAR-Frames/100023.pcd", *cloud2);

    // load transform matrix
    ifstream calib_result("/home/yao/Workspace/Livox_automatic_calibration/data/calibration_result.txt", ios::in);
    string lineStr;
    vector<vector<double>> calibration_result;
    while (getline(calib_result, lineStr))
    {
        stringstream ss(lineStr);
        ss.precision(10);
        string tmp;
        vector<double> temp_data;

        while (getline(ss, tmp, '\t'))
        {  //按“\t”隔开字符串
//            temp_data.push_back(std::stod(tmp)); //stod: string->double
            temp_data.push_back(boost::lexical_cast<double>(tmp));
        }

        calibration_result.push_back(temp_data);
        temp_data.clear();
    }

    Eigen::Matrix4d transform_matrix;
    for (int i = 0; i < calibration_result.size(); i++)
    {
        for (int j = 0; j < calibration_result[0].size(); j++)
        {
            transform_matrix(i, j) = calibration_result[i][j];
        }
    }

    // do transform
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud2, *transformed_cloud, transform_matrix);

    // combine point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr associatged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *associatged_cloud = *transformed_cloud + *cloud1;


    // viewer
    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
             "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud2, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
//    viewer.addPointCloud (cloud2, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
//    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> associated_cloud_color_handler (associatged_cloud, 20, 230, 20); // Red
    viewer.addPointCloud (associatged_cloud, associated_cloud_color_handler, "associated_cloud");

    viewer.addCoordinateSystem (1.0, 0);  //Adds 3D axes describing a coordinate system to screen at 0,0,0.
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "associated_cloud");

    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }


    // publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*associatged_cloud, output);
    output.header.frame_id = "map"; //
    associated_data_pub.publish(output);


    return 0;
}
