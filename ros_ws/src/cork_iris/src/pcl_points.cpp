#include <stdio.h>
// ROS Common
#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/point_types.h>
// ROS Sync
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
// ROS Msgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// Project 
#include "box.h"
#include "DepthParser.h"
#include "ImageParser.h"


/* Useful stuff

 - Adding a cube -> http://docs.pointclouds.org/1.8.1/classpcl_1_1visualization_1_1_p_c_l_visualizer.html#aa55f97ba784826552c9584d407014c78
 - Update pointcloud -> http://docs.pointclouds.org/1.8.1/classpcl_1_1visualization_1_1_p_c_l_visualizer.html#a629dbb72b085fa25d382fa982eaefa64
 
*/ 

using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
image_transport::Publisher parsed_pub;


// Temporary variable for debugging
bool temp_one_time_compute = false;
ros::Publisher pub;


void drawImageContours(cv::Mat drawing, std::vector<std::vector<cv::Point>> contours)
{
    for( int i = 0; i < contours.size(); i++ )
    {
        drawContours(drawing, contours, i, cv::Scalar(0, 255, 0), 1);
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normals_vis()
{   
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
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
    // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 70, 0.01, "normals");
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    // while (!viewer->wasStopped ())
    // {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // }
}


void process_image(cv::Mat image, cv::Mat depth){    

    cv::Mat parsed_image = image.clone();
    cv::Mat parsed_depth = depth.clone();
    
    // Image box
    cv::Mat box_image =  image.clone();
    Box box(box_image);
    std::vector<cv::Point> points = box.get_blue_box();
    // Get blue box painted all blue points bright red. Get the mask for all bright red points
    cv::Mat mask = box.getMaskInRange(cv::Scalar(0, 0, 250), cv::Scalar(0, 0, 255));

    // Contours
    int min_area = 20000;
    int max_area = 200000;
    std::vector<std::vector<cv::Point>> contour_points;
    ImageParser ip(mask);

    // This contour should be the inside contour (excluding the all the box around the cork pieces)
    // This is a heavy assumption since we are considering that only two contours exist after the first
    // area filter, the outer box cntour and the inner box contour.
    contour_points.push_back(ip.smallestAreaContour(ip.filterContoursByArea(ip.parseImageContours(-1), min_area, max_area)));
    drawImageContours(parsed_image, contour_points);

    sensor_msgs::ImagePtr msg; 
    try{
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", parsed_image).toImageMsg();
        parsed_pub.publish(msg);
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("ERROR: %s", e.what());
	    return; 
    }

}



void synced_callback(const sensor_msgs::ImageConstPtr& image, 
                    const sensor_msgs::ImageConstPtr& depth, 
                    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                    const sensor_msgs::CameraInfoConstPtr& depth_cam_info)
{

    cv_bridge::CvImagePtr cv_depth_ptr;
	cv_bridge::CvImagePtr cv_image_ptr;
    cv::Mat cv_image;
    cv::Mat cv_depth;

    try{
        // Convert sensor msg to cv mat
		cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    	cv_image_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
		cv_depth = cv_depth_ptr->image;
		cv_image = cv_image_ptr->image;

		// Convert depth enconding
		cv::Mat(cv_depth-0).convertTo(cv_depth, CV_8UC1, 255. / (1000 - 0));
		// sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", cv_depth).toImageMsg();
		// depth_pub.publish(msg);
	
	}catch(cv_bridge::Exception& e){
		ROS_ERROR("%s", e.what());
		return;
	}
	

    if(!temp_one_time_compute)
    {
        process_image(cv_image, cv_depth);

        temp_one_time_compute = true;
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

        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_x (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_x->width    = 640;
        cloud_x->height   = 480;
        cloud_x->is_dense = false;
        cloud_x->points.resize (cloud_x->width * cloud_x->height);

        for(int y = 0; y < cloud_x->height; y++){
            for(int x = 0; x < cloud_x->width; x++){
                pcl::PointXYZRGB point;
                point.x = cloud->at(x, y).x;
                point.y = cloud->at(x, y).y;
                point.z = cloud->at(x, y).z;
                point.r = abs(cloud_normals->at(x,y).normal[0]) * 255;
                point.g = abs(cloud_normals->at(x,y).normal[1]) * 255;
                point.b = abs(cloud_normals->at(x,y).normal[2]) * 255;

                cloud_x->points.push_back(point);
            }
        }

        temp_one_time_compute = true;
        std::cout << "Finished computing normals" << std::endl;

        setViewerPointcloud(cloud_x, cloud_normals);

    }

    viewerRunner(viewer);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_test");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
	parsed_pub = it.advertise("/corkiris/parsed", 1);
    // Creating subscribers for rgb, depth and cloud images, and syncing a callback for them

    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/camera/depth_registered/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(n, "/camera/depth_registered/points", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info(n, "/camera/depth_registered/camera_info", 1);
    
    using AproxSync = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                                     sensor_msgs::Image, 
                                                                     sensor_msgs::PointCloud2, 
                                                                     sensor_msgs::CameraInfo>;
    AproxSync mypolicy(32);

	message_filters::Synchronizer<AproxSync> sync{static_cast<const AproxSync &>(mypolicy), image_sub, depth_sub, pointcloud_sub, camera_info};
    sync.registerCallback(boost::bind(&synced_callback, _1, _2, _3, _4));

    viewer = normals_vis();


    // Create a ROS publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2> ("/newtopics/processed_depth", 1);
    ros::Rate loop_rate(10);
    // Spin
    ros::spin ();
}