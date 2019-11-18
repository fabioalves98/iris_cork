
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/point_types.h>


/* Useful stuff

 - Adding a cube -> http://docs.pointclouds.org/1.8.1/classpcl_1_1visualization_1_1_p_c_l_visualizer.html#aa55f97ba784826552c9584d407014c78
 - Update pointcloud -> http://docs.pointclouds.org/1.8.1/classpcl_1_1visualization_1_1_p_c_l_visualizer.html#a629dbb72b085fa25d382fa982eaefa64
 
*/ 

using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

// Temporary variable for debugging
bool temp_one_time_compute = false;
ros::Publisher pub;

boost::shared_ptr<pcl::visualization::PCLVisualizer> normals_vis()
{   
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

void setViewerPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, 
                         pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals)
{
    // USE UPADTEPOINTCLOUD METHOD TO SET THE POINT CLOUD
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 70, 0.01, "normals");

}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // PointXYZRGB cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Cloud normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());


    try{
        pcl::fromROSMsg (*cloud_msg, *cloud);
    }catch(pcl::PCLException& e){
		ROS_ERROR("%s", e.what());
		return;
	}

    if(!temp_one_time_compute)
    {
        std::cout << "Computing normals" << std::endl;
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud (cloud);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.03);
        ne.compute (*cloud_normals);

        setViewerPointcloud(cloud, cloud_normals);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_x (new pcl::PointCloud<pcl::PointXYZRGB>);

        for(int i = 200; i < 640; i++){
            for(int j = 200; j < 480; j++){
                cout << cloud->at(j, i) << endl;
                cout << cloud_normals->at(j, i) << endl;
            }
            break;
        }

        temp_one_time_compute = true;
        std::cout << "Finished computing normals" << std::endl;

        
    }

    viewerRunner(viewer);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_test");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);

    viewer = normals_vis();
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/newtopics/processed_depth", 1);
    ros::Rate loop_rate(10);
    // Spin
    ros::spin ();
}