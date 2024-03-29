#include <stdio.h>
#include <chrono>
#include <numeric>
#include <math.h>
#include <thread>
// ROS Common
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
// Dynamic Parameters Configure
#include <dynamic_reconfigure/server.h>
#include <iris_cork/PCLCorkConfig.h>
// ROS Sync
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
// ROS Msgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// Project 
#include "box.h"
#include "cork_iris.h"

#include <iris_sami/JointGoal.h>
#include <cork_classifier/ClassifyCork.h>

#include "iris_cork/CorkInfo.h"

using namespace std;

pcl::visualization::PCLVisualizer::Ptr viewer;


bool displayed = false;

// Global parameter values
int display_type;
bool live;
bool remove_stat_outliers;
bool smooth_cloud;
bool add_planning_scene_cork;
bool roller;
double grab_pos;

ros::Publisher pub;
ros::Publisher point_pub;
ros::Publisher aux_point_pub_1;
ros::Publisher aux_point_pub_2;
ros::Publisher cork_info;
ros::Publisher planning_scene_diff_publisher;
ros::ServiceClient classify_service;

CloudPtr cloud (new Cloud);

void drawBoundingBox(BoundingBox *bb, string name)
{
    viewer->removeShape(name);
    viewer->addCube(bb->position, bb->orientation, bb->maxPoint.x - bb->minPoint.x, bb->maxPoint.y - bb->minPoint.y, bb->maxPoint.z - bb->minPoint.z, name, 0);  
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, name);             
    viewer->setRepresentationToWireframeForAllActors(); 
}

pcl::visualization::PCLVisualizer::Ptr normalVis(string name)
{   
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (name));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    return (viewer);
}

void setViewerPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "kinectcloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "kinectcloud");
}

void setViewerPointcloudNormal(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, 
                               pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals)
{
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 70, 0.01, "normals");
}


CloudPtr cropBoundingBox(CloudPtr cloud, BoundingBox bb)
{
    CloudPtr cloudOut (new Cloud); 
   
    Eigen::Vector4f minPoint;
    minPoint[0]= bb.minPoint.x;
    minPoint[1]= bb.minPoint.y;
    minPoint[2]= bb.minPoint.z;
    Eigen::Vector4f maxPoint;
    maxPoint[0]= bb.maxPoint.x;
    maxPoint[1]= bb.maxPoint.y;  
    maxPoint[2]= bb.maxPoint.z;
    maxPoint[3] = 1;
    minPoint[3] = 1;

    Eigen::Affine3f trans = Eigen::Affine3f::Identity(); // create transform
    Eigen::Affine3f inverse_transform = Eigen::Affine3f::Identity(); 
    trans.translate(bb.position);                    
    trans.rotate(bb.orientation);  
    inverse_transform = trans.inverse (); 
    
    pcl::CropBox<pcl::PointXYZRGB> cropFilter;
    cropFilter.setInputCloud (cloud);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.setTransform(inverse_transform); 
    cropFilter.setKeepOrganized(true);
    cropFilter.filter (*cloudOut);

    return cloudOut; 

}

void publishCorkInfo(BoundingBox *bb, std::string classification)
{
    Eigen::Quaternionf corkOrientation = bb->orientation;

    geometry_msgs::Point center;
    center.x = bb->centroid.x();
    center.y = bb->centroid.y();
    center.z = bb->centroid.z();
    geometry_msgs::Quaternion orientation;
    orientation.x = corkOrientation.vec()[0];
    orientation.y = corkOrientation.vec()[1];
    orientation.z = corkOrientation.vec()[2];
    orientation.w = corkOrientation.w();
    geometry_msgs::PoseStamped cork_piece_pose;
    cork_piece_pose.header.stamp = ros::Time::now();
    cork_piece_pose.header.frame_id = "camera_depth_optical_frame";
    cork_piece_pose.pose.position = center;
    cork_piece_pose.pose.orientation = orientation;
    



    point_pub.publish(cork_piece_pose);

    //static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);  
    transform.setOrigin(tf::Vector3(center.x, center.y, center.z));
    transform.setRotation(q);
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "cork_piece"));
    geometry_msgs::TransformStamped tf_stamped;
    tf::StampedTransform transform_stamped(transform, ros::Time::now(), "camera_depth_optical_frame", "cork_piece");
    tf::transformStampedTFToMsg(transform_stamped, tf_stamped);

    iris_cork::CorkInfo cork_info_msg;
    cork_info_msg.transform = tf_stamped;
    cork_info_msg.classification = classification;
    cork_info.publish(cork_info_msg);

    
    // Adding cork piece object to move it planning scene
    if(add_planning_scene_cork){

        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "left_finger_link";
        attached_object.object.header.frame_id = "camera_depth_optical_frame";
        attached_object.object.id = "cork_piece";
        
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = bb->maxPoint.x - bb->minPoint.x;
        primitive.dimensions[1] = bb->maxPoint.y - bb->minPoint.y;
        primitive.dimensions[2] = bb->maxPoint.z - bb->minPoint.z;

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(cork_piece_pose.pose);
        attached_object.object.operation = attached_object.object.ADD;
        attached_object.touch_links = vector<string>{ "gripper_link", "left_finger_link", "right_finger_link" };

        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
    }

    

    double comp = bb->maxPoint.z - bb->minPoint.z;

    geometry_msgs::Point aux_center_1;
    aux_center_1.x = 0;
    aux_center_1.y = 0;
    aux_center_1.z = comp/2 - grab_pos;
    geometry_msgs::Point aux_center_2;
    aux_center_2.x = 0;
    aux_center_2.y = 0;
    aux_center_2.z = -(comp/2 - grab_pos);
    geometry_msgs::Quaternion aux_orientation;
    aux_orientation.x = 0;
    aux_orientation.y = 0;
    aux_orientation.z = 0;
    aux_orientation.w = 1;
    geometry_msgs::PoseStamped aux_cork_piece_1;
    aux_cork_piece_1.header.stamp = ros::Time::now();
    aux_cork_piece_1.header.frame_id = "camera_depth_optical_frame";
    aux_cork_piece_1.pose.position = aux_center_1;
    aux_cork_piece_1.pose.orientation = aux_orientation;
    geometry_msgs::PoseStamped aux_cork_piece_2 = aux_cork_piece_1;
    aux_cork_piece_2.pose.position = aux_center_2;

    geometry_msgs::TransformStamped cork_transform;
    cork_transform.header.stamp = ros::Time::now();
    cork_transform.header.frame_id = "camera_depth_optical_frame";
    cork_transform.child_frame_id = "cork_piece";
    cork_transform.transform.translation.x = bb->centroid.x();
    cork_transform.transform.translation.y = bb->centroid.y();
    cork_transform.transform.translation.z = bb->centroid.z();
    cork_transform.transform.rotation.x = corkOrientation.vec()[0];
    cork_transform.transform.rotation.y = corkOrientation.vec()[1];
    cork_transform.transform.rotation.z = corkOrientation.vec()[2];
    cork_transform.transform.rotation.w = q.w();

    tf2::doTransform(aux_cork_piece_1, aux_cork_piece_1, cork_transform);
    tf2::doTransform(aux_cork_piece_2, aux_cork_piece_2, cork_transform);

    aux_point_pub_1.publish(aux_cork_piece_1);
    aux_point_pub_2.publish(aux_cork_piece_2);
}

void parameterConfigure(iris_cork::PCLCorkConfig &config, uint32_t level) 
{
    // General params
    display_type = config.type;
    live = config.live;
    remove_stat_outliers = config.remove_outliers;
    smooth_cloud = config.smooth_cloud;
    add_planning_scene_cork = config.add_planning_scene_cork;
    roller = config.roller;
    grab_pos = config.grab_pos;

    PCLFunctions::updateParams(config);
    CorkIris::updateParams(config);

    displayed = false;
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

    try
    {
        // Convert sensor msg to cv mat
		cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    	cv_image_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
		cv_depth = cv_depth_ptr->image;
		cv_image = cv_image_ptr->image;

		// Convert depth enconding
		cv::Mat(cv_depth).convertTo(cv_depth, CV_8UC1, 255. / (1000));
	}
    catch(cv_bridge::Exception& e)
    {
		ROS_ERROR("%s", e.what());
		return;
	}

    try
    {
        pcl::fromROSMsg (*cloud_msg, *cloud);
    }
    catch(pcl::PCLException& e)
    {
		ROS_ERROR("%s", e.what());
		return;
	}
    
    if (!displayed || live)
    {   
        // Timer related stuff
        auto start = chrono::steady_clock::now();

        if (display_type == 0) // Original point cloud
        {
            viewer->updatePointCloud(cloud, "kinectcloud");
        }
        else // Original point cloud without the box
        { 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cork_pieces (new pcl::PointCloud<pcl::PointXYZRGB>);
            
            if(!roller)
            {
                Box::removeBox(cloud, cork_pieces, cv_image);
            }
            else
            {
                PCLFunctions::filterRollerBox(cloud, cork_pieces);
            }

            if (cork_pieces->size() == 0)
            {
                cout << "No Cork Pieces Found" << endl;
                viewer->removeShape("cork_piece");
            }
            else
            {          
                if (remove_stat_outliers) // Remove Statistical Outliers
                {
                    PCLFunctions::remove_outliers(cork_pieces, cork_pieces);
                }
                
                if (smooth_cloud) // Cloud smoothing
                {
                    PCLFunctions::cloud_smoothing(cork_pieces, cork_pieces);
                }
                
                if (display_type == 2) // Surface normals color
                {
                    PCLFunctions::surface_normals(cork_pieces, cork_pieces);
                }
                else if(display_type == 3) // Surface curvature color
                {
                    PCLFunctions::surface_curvatures(cork_pieces, cork_pieces);
                }
                else if (display_type == 4) // Clustering extraction
                {

                    CloudInfo cloud_cluster = CorkIris::clusterExtraction(cork_pieces, cork_pieces);
                    
                    CloudPtr cloud_cluster_full_res = cropBoundingBox(cloud, cloud_cluster.bb);
                    pcl::PCLImage pcl_image = PCLFunctions::extractImageFromCloud(cloud_cluster_full_res, true);
                    sensor_msgs::Image cloud_img;
                    pcl_conversions::moveFromPCL(pcl_image, cloud_img);
                    cork_classifier::ClassifyCork srv;
                    srv.request.cork_cloud = cloud_img;
                    srv.request.raw_image = *image;
                    classify_service.call(srv);

                    cout << srv.response.result << endl;

                    publishCorkInfo(&(cloud_cluster.bb), srv.response.result);
                    drawBoundingBox(&(cloud_cluster.bb), "cork_piece");

                    
                }
            }

            // Update the viewer and publish the processed pointclouds
            viewer->updatePointCloud(cork_pieces, "kinectcloud");

            sensor_msgs::PointCloud2 published_pcd;
            pcl::toROSMsg(*cork_pieces, published_pcd);
            published_pcd.header.frame_id = "camera_depth_optical_frame";
            pub.publish(published_pcd);            
        }

        auto end = chrono::steady_clock::now();
        auto diff = end - start;
        cout << "PointCloud processed in " << chrono::duration <double, milli> (diff).count() << " ms" << endl << endl  ;

        displayed = true; 
    }
    viewer->spinOnce (100);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "iris_cork");
    ros::NodeHandle n;

    // Creating subscribers for rgb, depth and cloud images, and syncing a callback for them
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/camera/depth_registered/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(n, "/camera/depth_registered/points", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info(n, "/camera/depth_registered/camera_info", 1);
    
    using AproxSync = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo>;
    AproxSync mypolicy(32);

    // Main callback with all the camera topics
	message_filters::Synchronizer<AproxSync> sync{static_cast<const AproxSync &>(mypolicy), image_sub, depth_sub, pointcloud_sub, camera_info};
    sync.registerCallback(boost::bind(&synced_callback, _1, _2, _3, _4));

    // Dynamic reconfigure init and callback
    dynamic_reconfigure::Server<iris_cork::PCLCorkConfig> server;
    dynamic_reconfigure::Server<iris_cork::PCLCorkConfig>::CallbackType pclConfigCallback;
    pclConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    server.setCallback(pclConfigCallback);
    
    // Initializing pcl viewer
    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud);

    // Create a ROS publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2> ("/iris_cork/processed_pointcloud", 1);
    point_pub = n.advertise<geometry_msgs::PoseStamped> ("/iris_cork/cork_piece", 1);
    aux_point_pub_1 = n.advertise<geometry_msgs::PoseStamped> ("/iris_cork/cork_piece_aux1", 1);
    aux_point_pub_2 = n.advertise<geometry_msgs::PoseStamped> ("/iris_cork/cork_piece_aux2", 1);
    cork_info = n.advertise<iris_cork::CorkInfo> ("/iris_cork/cork_info", 1);
    classify_service = n.serviceClient<cork_classifier::ClassifyCork>("/classify_cork");

    // ROS publisher for the planning scene
    planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    // Rate
    ros::Rate loop_rate(10);

    // Spin
    ros::spin ();
}