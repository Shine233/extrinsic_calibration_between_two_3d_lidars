//
// Created by yao on 10/11/21.
//

#include <lidar_calibration/data_sync.h>

using namespace std;

DataSync::DataSync()
{
    associated_data_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/livox/associated",1);
    lidar_sub1_.subscribe(nh_, "/livox/lidar_47MDJ790010062", 1);
    lidar_sub2_.subscribe(nh_, "/livox/lidar_47MDJ790010103", 1);
    sync_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), lidar_sub1_, lidar_sub1_);
    sync_->registerCallback(boost::bind(&DataSync::LivoxDataCallback, this, _1, _2));

    LoadTransformData();
}

DataSync::~DataSync()
{
    delete sync_;
}

void DataSync::LivoxDataCallback(const sensor_msgs::PointCloud2::ConstPtr &msg1,
                                 const sensor_msgs::PointCloud2::ConstPtr &msg2)
{
    sensor_msgs::PointCloud2::Ptr associatged_cloud(new sensor_msgs::PointCloud2);
    TransformAndCombinePointCloud(msg1, msg2, associatged_cloud);
    Publish(associatged_cloud);
}

void DataSync::LoadTransformData()
{
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
//            temp_data.push_back(std::stod(tmp)); //stod: string->double 会出现越界问题
            temp_data.push_back(boost::lexical_cast<double>(tmp));
        }

        calibration_result.push_back(temp_data);
        temp_data.clear();
    }


    for (int i = 0; i < calibration_result.size(); i++)
    {
        for (int j = 0; j < calibration_result[0].size(); j++)
        {
            transform_matrix_(i, j) = calibration_result[i][j];
        }
        cout<<endl;
    }
    cout<<"=========================" <<endl<<"The Transform matrix is: "<<endl
    <<transform_matrix_ <<endl<<"=========================" <<endl;
}

void DataSync::TransformAndCombinePointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg1,
                                             const sensor_msgs::PointCloud2::ConstPtr &msg2,
                                             sensor_msgs::PointCloud2::Ptr &output)
{
    // do transform
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg1(new pcl::PointCloud<pcl::PointXYZI>), pcl_msg2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg1, *pcl_msg1);
    pcl::fromROSMsg(*msg2, *pcl_msg2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*pcl_msg2, *transformed_cloud, transform_matrix_);

    // combine point cloud data
    pcl::PointCloud<pcl::PointXYZI>::Ptr associatged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *associatged_cloud = *transformed_cloud + *pcl_msg1;

    pcl::toROSMsg(*associatged_cloud, *output);
//    output->header.frame_id = "map";

    // viewer
    /*printf(  "\nPoint cloud colors :  white  = original point cloud\n"
             "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (pcl_msg2, 255, 255, 255);
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
    }*/
}

void DataSync::Publish(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    associated_data_pub_.publish(msg);
}