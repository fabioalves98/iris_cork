#include <stdio.h>
#include <chrono>
#include <numeric>
// ROS Common
#include <ros/ros.h>
// PCL specific includes
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/search/organized.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
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

#define DEBUG 0
#define SAVE_CLOUDS 0 


/* Useful stuff

 - Adding a cube -> http://docs.pointclouds.org/1.8.1/classpcl_1_1visualization_1_1_p_c_l_visualizer.html#aa55f97ba784826552c9584d407014c78
 
*/ 

using namespace std;

pcl::visualization::PCLVisualizer::Ptr viewer;
image_transport::Publisher parsed_pub;

bool displayed = false;

double normal_diff;
double squared_dist;
double curv;
bool smooth_cloud;

int num_enforce = 0;

ros::Publisher pub;

// PointXYZRGB cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
// Cloud normals
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
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

pcl::visualization::PCLVisualizer::Ptr normals_vis()
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

void removeBox(vector<cv::Point>* contours)
{
    const float bad_point = std::numeric_limits<float>::quiet_NaN();

    for (int x = 0; x < 640; x++)
    {
        for (int y = 0; y < 480; y++)
        {
            if (!isPointInside(cv::Point(x, y), contours))
            {
                cloud->at(x, y).x = cloud->at(x, y).y = cloud->at(x, y).z = bad_point;
            }
        }
    }
}

int getHighestPoint()
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

    kdtree.setInputCloud (cloud);

    pcl::PointXYZRGB searchPoint;

    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;

    // Changed here to one since we only need one point
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {  
        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
            cloud->points[pointIdxNKNSearch[i]].r = 0;
            cloud->points[pointIdxNKNSearch[i]].g = 255;
            cloud->points[pointIdxNKNSearch[i]].b = 0;
        }
    }

    // Indice 0 since it's just a point
    return pointIdxNKNSearch[0];

}

// void -> it draws the point cloud directly for now
// http://www.pointclouds.org/documentation/tutorials/#segmentation-tutorial
void findCorkPiece(){

    int highestPointIdx = getHighestPoint();
    pcl::PointXYZRGB highestPoint = cloud->points[highestPointIdx];
    // quanto menor o z mais alto ele esta
    // with kdtree get nearest points to the current one
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    int searchPointIdx = highestPointIdx;
    pcl::PointXYZRGB searchPoint = highestPoint;

    kdtree.setInputCloud (cloud);
    
    int K = 100 ;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::vector<int> closed_points;

    // Aqui para garantir que nao bloqueamos no while (que nao deve acontecer)
    // TODO: A ideia e agora adicionar algumas verificacoes com as normais
    // para garantir que selecionamos APENAS o traco e nao alguns "redores" dele
    int MAX_ITERS = 50;
    int iters = 0;
    while (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {  
        
        std::vector<int> aux_closed;
        // cout << "HighestPoint: " << cloud_normals->points[highestPointIdx] << endl;
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        {
            if (find(closed_points.begin(), closed_points.end(), pointIdxNKNSearch[i]) != closed_points.end()) continue;

            if (abs(searchPoint.z - cloud->points[pointIdxNKNSearch[i]].z) < 0.05)
            {    
                aux_closed.push_back(pointIdxNKNSearch[i]);
                // Paint the point. its a cork piece point
                cloud->points[pointIdxNKNSearch[i]].r = 0;
                cloud->points[pointIdxNKNSearch[i]].g = 255;
                cloud->points[pointIdxNKNSearch[i]].b = 0;       
            } 
        }

        cout << aux_closed.size() << endl;
        cout << searchPointIdx << endl;

        searchPointIdx = aux_closed.back();
        searchPoint = cloud->points[searchPointIdx];
        aux_closed.pop_back();
        closed_points.insert(closed_points.end(), aux_closed.begin(), aux_closed.end());

        // // Isto random esta aqui por agr. tem de ser alterado
        // int randIdx = rand() % pointIdxNKNSearch.size();
        // searchPointIdx = pointIdxNKNSearch[randIdx];
        pointIdxNKNSearch.clear();
        pointNKNSquaredDistance.clear();
        iters++;
        if(iters == MAX_ITERS) break;
    }
    
}

bool enforceNormals (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), 
    point_b_normal = point_b.getNormalVector3fMap ();

    num_enforce++;
    
    if(abs(point_a.curvature - point_b.curvature) > curv){
        return (false);
    }    
    
    if (squared_distance < squared_dist)
    {
        if (abs(point_a_normal.dot(point_b_normal)) > normal_diff){
            return (true);
        }
    }

    return (false);
}

void cluster_extraction (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    // Parametros do Algoritmo (Conditional Euclidean Clustering)
    double leaf_size = 0.005;
    //cout << "Leaf Size (0.005): " << endl;
    //cin >> leaf_size;
    double cluster_tolerance = 0.01;
    //cout << "Cluster Tolerance: " << endl;
    //cin >> cluster_tolerance;

    // Cloud downsampling -> mais eficiente e densidade de pontos mais equalizada
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
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_with_normals);
    
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&enforceNormals);
    cec.setClusterTolerance (cluster_tolerance);
    cec.setMinClusterSize (cloud_with_normals->points.size () / 70);
    cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);

    cout << "small cluster size: " << small_clusters->size() << endl;
    cout << "large cluster size: " << large_clusters->size() << endl;
    cout << "cluster size: " << clusters->size() << endl;
    cout << "original cloud size: " << cloud->width << " - " << cloud->height << endl;
    cout << "cloud filtered size: " << cloud_out->width  << " " << cloud_out->height<<  endl;
    cout << "number of comparissons: " << num_enforce << endl;

    for (int i = 0; i < small_clusters->size (); ++i)
    {
        for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
        {
            cloud_out->points[(*small_clusters)[i].indices[j]].r = 0;
            cloud_out->points[(*small_clusters)[i].indices[j]].g = 255;
            cloud_out->points[(*small_clusters)[i].indices[j]].b = 0;

        }
    }

    for (int i = 0; i < large_clusters->size (); ++i)
    {
        for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
        {
            cloud_out->points[(*large_clusters)[i].indices[j]].r = 0;
            cloud_out->points[(*large_clusters)[i].indices[j]].g = 0;
            cloud_out->points[(*large_clusters)[i].indices[j]].b = 255;

        }
    }

    for (int i = 0; i < clusters->size (); ++i)
    {
        int randR = rand() % 255;
        int randG = rand() % 255;
        int randB = rand() % 255;

        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
        {
            cloud_out->points[(*clusters)[i].indices[j]].r = randR;
            cloud_out->points[(*clusters)[i].indices[j]].g = randG;
            cloud_out->points[(*clusters)[i].indices[j]].b = randB;

        }
    }
}

void cloud_smoothing (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
    
    cout << "Starting cloud smoothing" << endl;
    
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

    cout << "Finished cloud smoothing" << endl;

    // Alterar tipo da cloud para PointXYZRGB
    copyPointCloud(mls_points, *cloud_out);
}

void surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    // Compute normals
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    cout << "Starting computing normals" << endl;
    ne.setInputCloud (cloud_in);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);
    cout << "Finished computing normals" << endl;
        
    cloud_out->width    = 640;
    cloud_out->height   = 480;
    cloud_out->is_dense = false;
    cloud_out->points.resize (cloud_out->width * cloud_out->height);

    for(int y = 0; y < cloud_out->height; y++)
    {
        for(int x = 0; x < cloud_out->width; x++)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->at(x, y).x;
            point.y = cloud->at(x, y).y;
            point.z = cloud->at(x, y).z;
            point.r = abs(cloud_normals->at(x,y).normal[0]) * 255;
            point.g = abs(cloud_normals->at(x,y).normal[1]) * 255;
            point.b = abs(cloud_normals->at(x,y).normal[2]) * 255;

            cloud_out->points.push_back(point);
        }
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
		cv::Mat(cv_depth).convertTo(cv_depth, CV_8UC1, 255. / (1000));
	
	}catch(cv_bridge::Exception& e){
		ROS_ERROR("%s", e.what());
		return;
	}

    
    try{
        pcl::fromROSMsg (*cloud_msg, *cloud);
    }catch(pcl::PCLException& e){
		ROS_ERROR("%s", e.what());
		return;
	}

    vector<cv::Point> corkContours = getCorkContours(cv_image);
    removeBox(&corkContours);

    if (!displayed)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        //surface_normals(cloud, cloud_out);

        if (smooth_cloud)
        {
            cloud_smoothing(cloud, cloud);
        }
        
        cluster_extraction(cloud, cloud_out);
        viewer->updatePointCloud(cloud_out, "kinectcloud");

        displayed = true;
    }
    viewer->spinOnce (100);
}

void getGlobalParams(ros::NodeHandle n){
    n.getParam("cork_iris/smooth_cloud", smooth_cloud);
    n.getParam("cork_iris/normal_diff", normal_diff);
    n.getParam("cork_iris/squared_dist", squared_dist);
    n.getParam("cork_iris/curvature", curv);
}

int main (int argc, char** argv)
{

    ros::init (argc, argv, "pcl_test");
    ros::NodeHandle n;
    getGlobalParams(n);


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
    setViewerPointcloud(cloud);

    // Create a ROS publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2> ("/newtopics/processed_depth", 1);
    ros::Rate loop_rate(10);
    // Spin
    ros::spin ();
}