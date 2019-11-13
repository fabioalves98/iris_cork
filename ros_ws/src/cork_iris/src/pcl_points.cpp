
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>


pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
ros::Publisher pub;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    try{
        pcl::fromROSMsg (*cloud_msg, *cloud);
    }catch(pcl::PCLException& e){
		ROS_ERROR("%s", e.what());
		return;
	}

    std::cout << "Starting normals computation" << std::endl;
    // Estimating Surface Normals
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    ne.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setRadiusSearch (0.03);

    ne.compute (*cloud_normals);
    std::cout << "Finalized normals computation" << std::endl;

    // sensor_msgs::PointCloud2 msg;
    // pcl::toROSMsg(*cloud_normals, msg);
    // std::cout << msg.header.frame_id << std::endl;
    // std::cout << msg.width<< std::endl;
    // pub.publish(msg);
    
    if(!viewer.wasStopped()){
        viewer.showCloud(cloud);
    }
    
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_test");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/newtopics/processed_depth", 1);
    ros::Rate loop_rate(10);
    // Spin
    ros::spin ();
}