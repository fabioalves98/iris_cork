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

//Dynamic Parameters Configure
#include <dynamic_reconfigure/server.h>
#include <cork_iris/PCLPointsConfig.h>

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


// Global parameter values
bool smooth_cloud;
bool live;
int display_type;

double normal_diff;
double squared_dist;
double curv;
double leaf_size;
double cluster_tolerance;
int min_cluster_size;
int max_cluster_size;

int num_enforce = 0;

ros::Publisher pub;

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

        //cout << aux_closed.size() << endl;
        //cout << searchPointIdx << endl;

        searchPointIdx = aux_closed.back();
        searchPoint = cloud->points[searchPointIdx];
        aux_closed.pop_back();
        closed_points.insert(closed_points.end(), aux_closed.begin(), aux_closed.end());

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

    double enf_normal_diff = point_a_normal.dot(point_b_normal);

    //cout << "\nNormal Difference: "  << enf_normal_diff << endl;      
    //cout << "Squared Distance: " << squared_distance << endl;
    
    if (squared_distance < squared_dist)
    {
        if (enf_normal_diff >= normal_diff)
        {
            return (true);
        }
    }

    return (false);
}

void cluster_extraction (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    //cout << "\nStaring Cluster Extraction" << endl;
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

    //cout << "Finished Cluster Extration\n" << endl;
}

void cloud_smoothing (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
    
    //cout << "\nStarting cloud smoothing" << endl;
    
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
    
    //cout << "Finished cloud smoothing\n" << endl;
    
    // Alterar tipo da cloud para PointXYZRGB
    copyPointCloud(mls_points, *cloud_out);
}

void surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
    // Compute normals
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    //cout << "\nStarting computing normals" << endl;
    ne.setInputCloud (cloud_in);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);
    //cout << "Finished computing normals\n" << endl;

    copyPointCloud(*cloud_in, *cloud_out);
        
    for (int i = 0; i < cloud_in->size(); i++)
    {
        cloud_out->points[i].r = abs(cloud_normals->points[i].normal[0]) * 255;
        cloud_out->points[i].g = abs(cloud_normals->points[i].normal[1]) * 255;
        cloud_out->points[i].b = abs(cloud_normals->points[i].normal[2]) * 255;
    }
}

void parameterConfigure(cork_iris::PCLPointsConfig &config, uint32_t level) 
{
    // General params
    display_type = config.type;
    live = config.live;
    smooth_cloud = config.smooth_cloud;

    // Clustering params
    leaf_size = config.leaf_size;
    cluster_tolerance = config.cluster_tolerance;
    
    min_cluster_size = config.min_cluster_size;
    max_cluster_size = config.max_cluster_size;
    normal_diff = config.normal_diff;
    squared_dist = config.squared_dist;
    curv = config.curvature;

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

            if (smooth_cloud) // Cloud smoothing
            {
                cloud_smoothing(cork_pieces, cork_pieces);
            }

            if (display_type == 2) // Surface normals color
            {
                surface_normals(cork_pieces, cork_pieces);
            }
            else if (display_type == 3) // Clustering extraction
            {
                cluster_extraction(cork_pieces, cork_pieces);
            }
            // Update the viewer and publish the processed pointcloud 
            viewer->updatePointCloud(cork_pieces, "kinectcloud");
            
            sensor_msgs::PointCloud2 published_pcd;
            pcl::toROSMsg(*cork_pieces, published_pcd);
            pub.publish(published_pcd);
        }

        auto end = chrono::steady_clock::now();
        auto diff = end - start;
        cout << "PointCloud processed in " << chrono::duration <double, milli> (diff).count() << " ms" << endl;

        displayed = true; 
    }
    viewer->spinOnce (100);
}

int main (int argc, char** argv)
{

    ros::init (argc, argv, "pcl_points");
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
    dynamic_reconfigure::Server<cork_iris::PCLPointsConfig> server;
    dynamic_reconfigure::Server<cork_iris::PCLPointsConfig>::CallbackType pclConfigCallback;
    pclConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    server.setCallback(pclConfigCallback);

	message_filters::Synchronizer<AproxSync> sync{static_cast<const AproxSync &>(mypolicy), image_sub, depth_sub, pointcloud_sub, camera_info};
    sync.registerCallback(boost::bind(&synced_callback, _1, _2, _3, _4));

    // Initializing pcl viewer
    viewer = normals_vis();
    setViewerPointcloud(cloud);

    // Create a ROS publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2> ("/cork_iris/processed_pointcloud", 1);
    ros::Rate loop_rate(10);
    // Spin
    ros::spin ();
}