#include <stdio.h>
#include <chrono>
#include <numeric>
#include <math.h>
// ROS Common
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// PCL specific includes
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/common/centroid.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>


//Dynamic Parameters Configure
#include <dynamic_reconfigure/server.h>
#include <cork_iris/PCLCorkConfig.h>

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
#include "DepthParser.h"
#include "ImageParser.h"


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef int Index;


struct BoundingBox{
    Eigen::Quaternionf orientation;
    Eigen::Vector3f position;
    pcl::PointXYZRGB minPoint, maxPoint;
    Eigen::Vector4f centroid;
};

struct CloudInfo{
    CloudPtr cloud;
    Eigen::Vector4f centroid;
};

pcl::visualization::PCLVisualizer::Ptr viewer;
image_transport::Publisher parsed_pub;

bool displayed = false;


// Global parameter values
int display_type;
bool live;
bool remove_stat_outliers;
bool smooth_cloud;
bool choose_best_cork;

double normal_diff;
double squared_dist;
double curv;
double leaf_size;
double cluster_tolerance;

double z_threshold, center_threshold;

double radius_search;

int min_cluster_size;
int max_cluster_size;

int meanK;

int num_enforce = 0;

ros::Publisher pub;
ros::Publisher point_pub;

// PointXYZRGB cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
// Cloud normals
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), 
small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);

void drawImageContours(cv::Mat drawing, std::vector<std::vector<cv::Point>> contours)
{
    for( int i = 0; i < contours.size(); i++ )
    {
        drawContours(drawing, contours, i, cv::Scalar(0, 255, 0), 1);
    }
}

pcl::visualization::PCLVisualizer::Ptr normalsVis()
{   
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
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

vector<cv::Point> getCorkContours(cv::Mat image)
{        
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
    
    return contour_points.at(0);
}

bool isPointInside(cv::Point point, std::vector<cv::Point>* contour)
{
    return cv::pointPolygonTest(*contour, point, false) == 1;
}

void removeBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
               vector<cv::Point>* contours)
{
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    copyPointCloud(*cloud_in, *cloud_out);

    for (int x = 0; x < cloud_in->width; x++)
    {
        for (int y = 0; y < cloud_in->height; y++)
        {
            if (!isPointInside(cv::Point(x, y), contours))
            {
                cloud_out->at(x, y).x = cloud_out->at(x, y).y = cloud_out->at(x, y).z = bad_point;
            }
        }
    }
}


bool enforceNormals (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), 
    point_b_normal = point_b.getNormalVector3fMap ();

    num_enforce++;

    double enf_normal_diff = point_a_normal.dot(point_b_normal);
    
    if (squared_distance < squared_dist)
    {
        if (enf_normal_diff >= normal_diff)
        {
            if (point_b.curvature < curv)
            {
                return (true);
            }   
        }
    }

    return (false);
}

std::vector<CloudInfo> clusterIndicesToCloud(pcl::IndicesClustersPtr clusters, CloudPtr original_cloud)
{
    std::vector<CloudInfo> cloud_clusters;
    for(int i = 0; i < clusters->size(); i++)
    {
        CloudPtr cloud_cluster (new Cloud);
        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
        {
            cloud_cluster->push_back(original_cloud->points[(*clusters)[i].indices[j]]);
        }
        CloudInfo cloud_info;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*(cloud_cluster), centroid);
        cloud_info.cloud = cloud_cluster;
        cloud_info.centroid = centroid;
        cloud_clusters.push_back(cloud_info);  
    }
    return cloud_clusters;
}


BoundingBox computeCloudBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_in, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_in, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    ///    the signs are different and the box doesn't get correctly oriented in some cases.
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
                                                                               
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_in, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    BoundingBox bb;
    bb.orientation = bboxQuaternion;
    bb.position = bboxTransform;
    bb.minPoint = minPoint;
    bb.maxPoint = maxPoint;
    bb.centroid = pcaCentroid;

    return bb;

}

void drawBoundingBox(BoundingBox *bb)
{
    viewer->removeShape("bbox");
    viewer->addCube(bb->position, bb->orientation, bb->maxPoint.x - bb->minPoint.x, bb->maxPoint.y - bb->minPoint.y, bb->maxPoint.z - bb->minPoint.z, "bbox", 0);  
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "bbox");             
    viewer->setRepresentationToWireframeForAllActors(); 
}

void broadcastCorkTransform(BoundingBox *bb)
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

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(center.x, center.y, center.z));
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);  
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "cork_piece"));

}


Index getHighestCluster(std::vector<CloudInfo> clusters)
{
    if(clusters.size() == 0) return -1;

    Index idx = 0;
    float highest = clusters[0].centroid.z();
    for(int i = 1; i < clusters.size(); i++)
    {
        if(clusters[i].centroid.z() <= highest)
        {
            highest = clusters[i].centroid.z();
            idx = i;
        }
    }
    return idx;
}

Index getMostCentralCluster(std::vector<CloudInfo> clusters)
{
    if(clusters.size() == 0) return -1;

    Index idx = 0;
    float min_dist = sqrt((clusters[0].centroid.x() * clusters[0].centroid.x()) + (clusters[0].centroid.y() * clusters[0].centroid.y()));
    for(int i = 1; i < clusters.size(); i++)
    {
        float dist = sqrt((clusters[i].centroid.x() * clusters[i].centroid.x()) + (clusters[i].centroid.y() * clusters[i].centroid.y()));
        if(dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }
    return idx;
}



CloudPtr chooseBestCluster(pcl::IndicesClustersPtr clusters, CloudPtr fullPointCloud)
{
    // cout << clusters << endl;
    // cout << fullPointCloud << endl;
    auto start = chrono::steady_clock::now();


    Index idx = 0;
    std::vector<CloudInfo> cluster_clouds = clusterIndicesToCloud(clusters, fullPointCloud);
    cout << "Got cloudinfo" << cluster_clouds.size() << endl;
    if(!choose_best_cork && cluster_clouds.size() > 0){
        return cluster_clouds[idx].cloud;    
    }

    Index highest_cloud_idx = getHighestCluster(cluster_clouds);
    Index closest_cloud_idx = getMostCentralCluster(cluster_clouds);

    if(highest_cloud_idx < 0) {cout << "FAILED HIGHEST" << endl;}
    if(closest_cloud_idx < 0) {cout << "FAILED CLOSEST" << endl;}

    CloudInfo highest_cloud = cluster_clouds[highest_cloud_idx];
    double THRESHOLD_Z = highest_cloud.centroid.z() + z_threshold;
    CloudInfo closest_center = cluster_clouds[closest_cloud_idx];
    double THRESHOLD_CENTER = center_threshold;//sqrt((closest_center.centroid.x() * closest_center.centroid.x()) + (closest_center.centroid.y() * closest_center.centroid.y())) + 0.05;

    for(int i = 0; i < cluster_clouds.size(); i++){
        if(cluster_clouds[i].centroid.z() <= THRESHOLD_Z) // or > threshold_z - 2*z_threshold
        {
            cout << "Cluster " << i << " passed z thresh" << endl;
            float dist2center = sqrt((cluster_clouds[i].centroid.x() * cluster_clouds[i].centroid.x()) + (cluster_clouds[i].centroid.y() * cluster_clouds[i].centroid.y())); 
            cout << "THRESHOLD CENTER: " << THRESHOLD_CENTER << endl << "Dist2Center: " << dist2center << endl;
            if(dist2center <= THRESHOLD_CENTER)
            {
                cout << "Cluster " << i << " might be the chosen one!" << endl;
                idx = i;
            }
            cout << "----" << endl;
        }
        
    }

    auto end = chrono::steady_clock::now();
    auto diff = end - start;
    cout << "Picked best cork in " << chrono::duration <double, milli> (diff).count() << " ms" << endl << endl  ;

    return cluster_clouds[idx].cloud;
}


void cluster_extraction (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{


    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud_out);
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_out, *cloud_with_normals);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (cloud_out);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_with_normals);
    
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&enforceNormals);
    cec.setClusterTolerance (cluster_tolerance);
    cec.setMinClusterSize (cloud_with_normals->points.size () / min_cluster_size);
    cec.setMaxClusterSize (cloud_with_normals->points.size () / max_cluster_size);
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);

    //cout << "small cluster size: " << small_clusters->size() << endl;
    //cout << "large cluster size: " << large_clusters->size() << endl;
    //cout << "cluster size: " << clusters->size() << endl;
    //cout << "original cloud size: " << cloud->width << " - " << cloud->height << endl;
    //cout << "cloud filtered size: " << cloud_out->width  << " " << cloud_out->height<<  endl;
    //cout << "number of comparissons: " << num_enforce << endl;

    for (int i = 0; i < small_clusters->size (); ++i)
    {
        for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
        {
            cloud_out->points[(*small_clusters)[i].indices[j]].r = 100;
            cloud_out->points[(*small_clusters)[i].indices[j]].g = 100;
            cloud_out->points[(*small_clusters)[i].indices[j]].b = 100;

        }
    }
    

    for (int i = 0; i < large_clusters->size (); ++i)
    {
        for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
        {
            cloud_out->points[(*large_clusters)[i].indices[j]].r = 100;
            cloud_out->points[(*large_clusters)[i].indices[j]].g = 50;
            cloud_out->points[(*large_clusters)[i].indices[j]].b = 0;
        }
    }

    if(clusters->size() == 0){
        cout << "No clusters found. Returning!" << endl;
        return;
    }

    for (int i = 0; i < clusters->size (); ++i)
    {
        int randR = rand() % 255;
        int randG = rand() % 255;
        int randB = rand() % 255;
      
        bitset<8> binary = bitset<8>(i);

        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
        {
            if (i % 8 == 0) 
            {
                cloud_out->points[(*clusters)[i].indices[j]].r = 255;
                cloud_out->points[(*clusters)[i].indices[j]].g = 150;
                cloud_out->points[(*clusters)[i].indices[j]].b = 0;
            }
            else
            {
                cloud_out->points[(*clusters)[i].indices[j]].r = 255 * binary[0];
                cloud_out->points[(*clusters)[i].indices[j]].g = 255 * binary[1];
                cloud_out->points[(*clusters)[i].indices[j]].b = 255 * binary[2];
            }   
        }
    }

 
    // Transforming the cluster into a cloud
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    // for (int j = 0; j < (*clusters)[0].indices.size (); ++j)
    // {
    //     cloud_cluster->push_back(cloud_out->points[(*clusters)[0].indices[j]]);
    // }

    CloudPtr cloud_cluster = chooseBestCluster(clusters, cloud_out);


    BoundingBox cork_piece = computeCloudBoundingBox(cloud_cluster);
    broadcastCorkTransform(&cork_piece);
    drawBoundingBox(&cork_piece);
    // drawCloudBoundingBox(cloud_cluster);

    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);

    pcl::PointXYZRGB painted;
    painted.x = pcaCentroid.x();
    painted.y = pcaCentroid.y();
    painted.z = pcaCentroid.z(); 

    painted.r = 0;
    painted.g = 0;
    painted.b = 255;

    cloud_out->push_back(painted);
}

void remove_outliers (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_out);
}

void cloud_smoothing (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
        
    mls.setComputeNormals (true);
    // Set parameters
    vector<int> test;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, test);
    mls.setInputCloud (cloud_in);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (mls_points);
        
    // Alterar tipo da cloud para PointXYZRGB
    copyPointCloud(mls_points, *cloud_out);
}

void surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    // Compute normals
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud (cloud_in);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_normals);

    copyPointCloud(*cloud_in, *cloud_out);
        
    for (int i = 0; i < cloud_in->size(); i++)
    {
        cloud_out->points[i].r = abs(cloud_normals->points[i].normal[0]) * 255;
        cloud_out->points[i].g = abs(cloud_normals->points[i].normal[1]) * 255;
        cloud_out->points[i].b = abs(cloud_normals->points[i].normal[2]) * 255;
    }
}

void surface_curvatures (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    // Compute normals
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud (cloud_in);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_normals);

    copyPointCloud(*cloud_in, *cloud_out);
        
    for (int i = 0; i < cloud_in->size(); i++)
    {
        cloud_out->points[i].r = cloud_out->points[i].g = cloud_out->points[i].b = 55 + abs(cloud_normals->points[i].curvature / 0.3) * 200;
    }
}

void parameterConfigure(cork_iris::PCLCorkConfig &config, uint32_t level) 
{
    // General params
    display_type = config.type;
    live = config.live;
    remove_stat_outliers = config.remove_outliers;
    smooth_cloud = config.smooth_cloud;
    choose_best_cork = config.choose_best_cork;

    // Best cork algorithm params
    z_threshold = config.z_threshold;
    center_threshold = config.center_threshold;

    radius_search = config.radius_search;

    // Clustering params
    leaf_size = config.leaf_size;
    cluster_tolerance = config.cluster_tolerance;
    min_cluster_size = config.min_cluster_size;
    max_cluster_size = config.max_cluster_size;
    normal_diff = config.normal_diff;
    squared_dist = config.squared_dist;
    curv = config.curvature;

    // Statistical outliers params
    meanK = config.mean_k;
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
            vector<cv::Point> corkContours = getCorkContours(cv_image);
            removeBox(cloud, cork_pieces, &corkContours);

            if (remove_stat_outliers) // Remove Statistical Outliers
            {
                remove_outliers(cork_pieces, cork_pieces);
            }
            if (smooth_cloud) // Cloud smoothing
            {
                cloud_smoothing(cork_pieces, cork_pieces);
            }
            if (display_type == 2) // Surface normals color
            {
                surface_normals(cork_pieces, cork_pieces);
            }
            else if(display_type == 3) // Surface curvature color
            {
                surface_curvatures(cork_pieces, cork_pieces);
            }
            else if (display_type == 4) // Clustering extraction
            {
                cluster_extraction(cork_pieces, cork_pieces);
            }

            // Update the viewer and publish the processed pointcloud 
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
    ros::init (argc, argv, "pcl_cork");
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

    // Dynamic reconfigure init and callback
    dynamic_reconfigure::Server<cork_iris::PCLCorkConfig> server;
    dynamic_reconfigure::Server<cork_iris::PCLCorkConfig>::CallbackType pclConfigCallback;
    pclConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    server.setCallback(pclConfigCallback);

	message_filters::Synchronizer<AproxSync> sync{static_cast<const AproxSync &>(mypolicy), image_sub, depth_sub, pointcloud_sub, camera_info};
    sync.registerCallback(boost::bind(&synced_callback, _1, _2, _3, _4));

    // Initializing pcl viewer
    viewer = normalsVis();
    setViewerPointcloud(cloud);

    // Create a ROS publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2> ("/cork_iris/processed_pointcloud", 1);
    point_pub = n.advertise<geometry_msgs::PoseStamped> ("/cork_iris/cork_piece", 1);
    ros::Rate loop_rate(10);
    // Spin
    ros::spin ();
}