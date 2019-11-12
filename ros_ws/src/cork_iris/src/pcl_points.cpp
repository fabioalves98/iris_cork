
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    try{
        pcl::fromROSMsg (*cloud_msg, *cloud);
    }catch(pcl::PCLException& e){
		ROS_ERROR("%s", e.what());
		return;
	}

    if (!viewer.wasStopped ())
    {
        viewer.showCloud (cloud);
    }
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_test");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<sensor_msgs::PointCloud2> ("/newtopics/processed_depth", 1);
    
    // Spin
    ros::spin ();
}