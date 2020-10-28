#include <stdio.h>
#include <chrono>
#include <numeric>
#include <math.h>
#include <thread>

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
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/common/centroid.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

// moveit
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>


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
#include "pcl_functions.h"


using namespace std;
// typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
// typedef Cloud::Ptr CloudPtr;
// typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudNormal;
// typedef CloudNormal::Ptr CloudNormalPtr;
// typedef pcl::RGB Color;
typedef int Index;


struct BoundingBox{
    Eigen::Quaternionf orientation;
    Eigen::Vector3f position;
    pcl::PointXYZRGB minPoint, maxPoint;
    Eigen::Vector4f centroid;
};

struct CloudInfo{
    CloudPtr cloud;
    CloudNormalPtr cloudNormal;
    pcl::PointIndices::Ptr indices;
    BoundingBox bb;
};

struct CECExtractionParams{
    double normal_diff;
    double squared_dist;
    double curv;
    double leaf_size;
    double cluster_tolerance;
    double radius_search;
    int min_cluster_size, max_cluster_size;
};

pcl::visualization::PCLVisualizer::Ptr viewer;


// Debug variable to get color from array index, since cluster colors are based on indices
std::vector<std::string> colors = {"orange", "red", "green", "yellow", "blue", "pink", "cyan"};

bool displayed = false;

// Global parameter values
int display_type;
bool live;
bool remove_stat_outliers;
bool smooth_cloud;
bool choose_best_cork;
bool add_planning_scene_cork;

bool roller;
double roller_height_value;
double roller_height_angle;
double roller_width_value;

CECExtractionParams cecparams;

// int min_cluster_size, max_cluster_size;

int meanK;

double z_threshold, center_threshold;
double space_distance_threshold, space_count_points_threshold, space_k_neighbors;
double bad_shape_percentage_threshold, volume_threshold;
double splitted_cork_distance_threshold, splitted_cork_normal_threshold;


// End global parameter values

ros::Publisher pub;
ros::Publisher point_pub;
ros::Publisher cork_cloud;
ros::Publisher planning_scene_diff_publisher;

// PointXYZRGB cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


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


void getNearestNeighbors(int K, Eigen::Vector4f searchPoint, CloudPtr cloud, vector<int>& points, vector<float>& dists)
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (cloud);
    
    // vector<int> nearestPointIndices(K);
    // vector<float> nearestPointDistances(K);

    cout << searchPoint.x() << " " << searchPoint.y() << " " << searchPoint.z() << endl;

    // pcl::PointXYZRGB startingPoint(searchPoint.x(), searchPoint.y(), searchPoint.z());
    pcl::PointXYZRGB startingPoint;
    startingPoint.x = searchPoint.x();
    startingPoint.y = searchPoint.y();
    startingPoint.z = searchPoint.z(); 

    kdtree.nearestKSearch (startingPoint, K, points, dists);
}


BoundingBox computeCloudBoundingBox(CloudPtr cloud_in)
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
    // MaxPoint - MinPoint
    // x -> altura da caixa
    // y -> largura da caixa
    // z -> comprimento da caixa
    bb.centroid = pcaCentroid;

    return bb;

}

/*

    Returns an array of CloudInfo objects containing the cloud representing each individual cluster and
    the respective BoundingBox (Including centroid, min and max point etc...). It also saves the PointIndices
    related to the cluster

*/
std::vector<CloudInfo> clusterIndicesToCloud(pcl::IndicesClustersPtr clusters, CloudPtr original_cloud, CloudNormalPtr original_cloud_normal)
{
    std::vector<CloudInfo> cloud_clusters;
    for(int i = 0; i < clusters->size(); i++)
    {
        CloudPtr cloud_cluster (new Cloud);
        CloudNormalPtr cloud_normal_cluster (new CloudNormal);
        pcl::PointIndices::Ptr cloud_indices (new pcl::PointIndices);
        cloud_indices->indices = (*clusters)[i].indices;
        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
        {
            cloud_cluster->push_back(original_cloud->points[(*clusters)[i].indices[j]]);
            cloud_normal_cluster->push_back(original_cloud_normal->points[(*clusters)[i].indices[j]]);
        }
        CloudInfo cloud_info;
        cloud_info.cloud = cloud_cluster;
        cloud_info.cloudNormal = cloud_normal_cluster;
        cloud_info.bb = computeCloudBoundingBox(cloud_cluster);
        cloud_info.indices = cloud_indices;
        cloud_clusters.push_back(cloud_info);  
    }
    return cloud_clusters;
}




CloudPtr subtractCloud(CloudPtr cloud, pcl::PointIndices::Ptr indices)
{
    CloudPtr cloud_subtracted (new Cloud);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (indices);
    extract.setNegative (true);
    extract.filter (*cloud_subtracted);

    return cloud_subtracted;

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
    cropFilter.filter (*cloudOut);

    return cloudOut; 

}


/*
    Computes an average normal associated with a cluster.
*/
Eigen::Vector3f getClusterAverageNormal(CloudNormalPtr cluster)
{
    Eigen::Vector3f currentNormal = cluster->points[0].getNormalVector3fMap();
    for(int i = 1; i < cluster->points.size(); i++)
    {
        currentNormal += cluster->points[i].getNormalVector3fMap();
    }

    return currentNormal / cluster->points.size();

}

CloudInfo joinClusters(CloudInfo cluster0, CloudInfo cluster1)
{
    cout << "Cloud b4:" << cluster0.cloud->size() << endl;
    *(cluster0.cloud) += *(cluster1.cloud);
    *(cluster0.cloudNormal) += *(cluster1.cloudNormal);
    (cluster0.indices->indices).insert(cluster0.indices->indices.end(), cluster1.indices->indices.begin(), cluster1.indices->indices.end());
    if(cluster1.bb.centroid.z() > cluster0.bb.centroid.z())
        cluster0.bb = cluster1.bb;
    cout << "Centrofd BB0: " << cluster0.bb.centroid << endl << "Centroid BB1: " << cluster1.bb.centroid << endl;
    cout << "Cloud after:" << cluster0.cloud->size() << endl;


    return cluster0;
}


void paintClusters(CloudPtr cloud_out, std::vector<CloudInfo> clusters)
{
    for (int i = 0; i < clusters.size(); ++i)
    {      
        bitset<8> binary = bitset<8>(i);

        for (int j = 0; j < clusters[i].indices->indices.size (); ++j)
        {
            if (i % 8 == 0) 
            {
                cloud_out->points[clusters[i].indices->indices[j]].r = 255;
                cloud_out->points[clusters[i].indices->indices[j]].g = 150;
                cloud_out->points[clusters[i].indices->indices[j]].b = 0;
            }
            else
            {
                cloud_out->points[clusters[i].indices->indices[j]].r = 255 * binary[0];
                cloud_out->points[clusters[i].indices->indices[j]].g = 255 * binary[1];
                cloud_out->points[clusters[i].indices->indices[j]].b = 255 * binary[2];
            }   
        }
    }  
}


void paintClustersFull(CloudPtr cloud_out, std::vector<CloudInfo> clusters, pcl::IndicesClustersPtr small_clusters, pcl::IndicesClustersPtr large_clusters)
{
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


    paintClusters(cloud_out, clusters);

}

void paintPoints(CloudPtr cloud_out, vector<int> points, Color color)
{
    for (int i = 0; i < points.size (); ++i)
    {
        (*cloud_out)[points[i]].r = color.r;
        (*cloud_out)[points[i]].g = color.g;
        (*cloud_out)[points[i]].b = color.b;
    }
}




bool enforceNormals (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), 
    point_b_normal = point_b.getNormalVector3fMap ();


    double enf_normal_diff = point_a_normal.dot(point_b_normal);
    if (squared_distance < cecparams.squared_dist)
    {
        if (enf_normal_diff >= cecparams.normal_diff)
        {
            return (true);
            // if (point_b.curvature < cecparams.curv)
            // {
            //     return (true);
            // }   
        }
    }

    return (false);
}


/*
    Segments the cloud_in using CEC and ultimately returns a new processed cloud, indices for clusters, small clusters and big clusters.
    Also returns the cloud with normals estimated during the process
*/

void CECExtraction(CloudPtr cloud_in, CloudPtr cloud_out, 
                   pcl::IndicesClustersPtr clusters, pcl::IndicesClustersPtr& sclusters, pcl::IndicesClustersPtr& lclusters,
                   CloudNormalPtr cloud_with_normals)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (cecparams.leaf_size, cecparams.leaf_size, cecparams.leaf_size);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud_out);
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*cloud_out, *cloud_with_normals);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (cloud_out);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (cecparams.radius_search);
    ne.compute (*cloud_with_normals);

    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&enforceNormals);
    cec.setClusterTolerance (cecparams.cluster_tolerance);
    cec.setMinClusterSize (cloud_with_normals->points.size () / cecparams.min_cluster_size);
    cec.setMaxClusterSize (cloud_with_normals->points.size () / cecparams.max_cluster_size);
    cec.segment (*clusters);
    cec.getRemovedClusters (sclusters, lclusters);

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
        attached_object.touch_links = std::vector<std::string>{ "ee_link", "left_finger_link", "right_finger_link" };

        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(center.x, center.y, center.z));
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);  
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "cork_piece"));

}




/*
    Determines if the shape of the cluster is "cork" like
*/
bool isClusterBadShaped(BoundingBox cluster)
{
    float THRESHOLD_PERCENTAGE = bad_shape_percentage_threshold;
    float largura = cluster.maxPoint.y - cluster.minPoint.y;
    float comprimento = cluster.maxPoint.z - cluster.minPoint.z;

    return ((largura/comprimento)) > THRESHOLD_PERCENTAGE;
}

/*
    Determines if the cluster bounding box is too big to be a "cork_piece"
*/
bool isClusterTooBig(BoundingBox cluster)
{
    float THRESHOLD_VOLUME = volume_threshold;
    float largura = cluster.maxPoint.y - cluster.minPoint.y;
    float comprimento = cluster.maxPoint.z - cluster.minPoint.z;
    float altura = cluster.maxPoint.x - cluster.minPoint.x;
    return (largura * comprimento * altura) > THRESHOLD_VOLUME;
}



/*
    Determines if a cork_piece was splitted into two different clusters (Works best if the clusters inserted here are one after the other
    in the original cluster array, since this means they were clustered one after the other. Clusters where this didn't happend should belong
    to the same cork_piece).
*/
bool isSplittedCluster(CloudInfo cluster0, CloudInfo cluster1)
{

    double THRESHOLD_SPLIT_DISTANCE = splitted_cork_distance_threshold;
    Eigen::Vector4f centroid0 = cluster0.bb.centroid;
    Eigen::Vector4f centroid1 = cluster1.bb.centroid;
    double distance = sqrt(pow((centroid0.x() - centroid1.x()), 2) + pow((centroid0.y() - centroid1.y()), 2) + pow((centroid0.z() - centroid1.z()), 2));
    cout << "Distance between clusters: " << distance << endl; 
    if(distance > THRESHOLD_SPLIT_DISTANCE)
        return false;

    double THRESHOLD_DOT_PRODUCT = splitted_cork_normal_threshold; 
    Eigen::Vector3f avg_normal0 = getClusterAverageNormal(cluster0.cloudNormal);
    Eigen::Vector3f avg_normal1 = getClusterAverageNormal(cluster1.cloudNormal);
    double dot_product = avg_normal0.dot(avg_normal1);

    cout << "Dot product: " << dot_product << endl;
    // Check if there are a bunch of points in a common line between them
    return dot_product < THRESHOLD_DOT_PRODUCT;

}

vector<CloudInfo> joinSplittedClusters(vector<CloudInfo> clusters)
{
    vector<CloudInfo> new_vec;
    cout << "<<<<< JOINING SPLITTED CLUSTERS >>>>>>" << endl;
    if (clusters.size() == 1)
    {
        return clusters;
    }
    for(int i = 0; i < clusters.size()-1; i++)
    {
        if(isSplittedCluster(clusters[i], clusters[i+1]))
        {
            cout << "Join " << colors[i%7] << " with " << colors[(i+1)%7] << endl; 
            CloudInfo new_cluster = joinClusters(clusters[i], clusters[i+1]);
            new_vec.push_back(new_cluster);
            i++; // Skip next cluster that was joined with i
        }else{
            new_vec.push_back(clusters[i]);
            if(i == clusters.size()-2) new_vec.push_back(clusters[i+1]);
        }
    }
    cout << "ENDED JOINING SPLITTED CLUSTERS" << endl; 
    return new_vec;
}

/*
    Runs cluster extraction again in the sub clusters that are considered too big or out of shape
*/
vector<CloudInfo> segmentBigClusters(vector<CloudInfo> clusters)
{
    vector<CloudInfo> new_vec;
    cout << "<<<<< SEGMENTING BIG/OUT OF SHAPE CLUSTERS >>>>>>" << endl;
    CloudPtr display_cloud (new Cloud);
    for(int i = 0; i < clusters.size(); i++)
    {
        if(isClusterBadShaped(clusters[i].bb) || isClusterTooBig(clusters[i].bb))
        {
            cout << "Cluster " << colors[i%7] << " is either too big or bad shaped. Segmenting again" << endl;
            // CECExtraction(clusters[i].cloud, )
            // CloudNormalPtr cloud_with_normals (new CloudNormal);
            // pcl::IndicesClustersPtr clusts (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
            // CloudPtr cloud_out (new Cloud);
            
            // // Save params in aux, change them to improve segmentation
            // CECExtractionParams aux = cecparams;
            // cecparams.leaf_size = 0.0095;
            // cecparams.squared_dist = test_param2;
            // // cecparams.normal_diff = 0.997;
            // cecparams.normal_diff = test_param;
            // cecparams.min_cluster_size = 200;


            // CECExtraction(clusters[i].cloud, cloud_out, clusts, small_clusters, large_clusters, cloud_with_normals);
            // cecparams = aux;
            // Bring back the original params

            // cout << "cloud info clusters: " << clusts->size() << endl;
            // cout << "cloud info clusters small : " << small_clusters->size() << endl;
            // cout << "cloud info clusters big: " << large_clusters->size() << endl;
            // std::vector<CloudInfo> cloud_info_clusters = clusterIndicesToCloud(clusts, cloud_out, cloud_with_normals);
            // paintClusters(cloud_out, cloud_info_clusters);
            // *(display_cloud) += *(cloud_out);

            // setup bb etc etc...
        }else{
            new_vec.push_back(clusters[i]);
        }

    }
    // viewer->updatePointCloud(display_cloud, "kinectcloud");


    cout << "ENDED SEGMENTATION" << endl; 
    return new_vec;


}


Index getHighestCluster(std::vector<CloudInfo> clusters)
{
    if(clusters.size() == 0) return -1;

    Index idx = 0;
    float highest = clusters[0].bb.centroid.z();
    for(int i = 1; i < clusters.size(); i++)
    {
        if(clusters[i].bb.centroid.z() <= highest)
        {
            highest = clusters[i].bb.centroid.z();
            idx = i;
        }
    }
    return idx;
}


bool isThereSpace(CloudInfo cluster, CloudPtr fullCloud)
{

    CloudPtr fullCloudNoCluster = subtractCloud(fullCloud, cluster.indices);
    
    int K = space_k_neighbors;
    float DISTANCE_THRESHOLD_COUNT = space_distance_threshold;
    int COUNT_THRESHOLD = space_count_points_threshold;

    vector<int> points(K);
    vector<float> distances(K);
    getNearestNeighbors(K, cluster.bb.centroid, fullCloudNoCluster, points, distances);
    int counter = 0;
    for(int i = 0; i < points.size(); i++)
    {
        // if(i < 30) cout << distances[i] << endl;
        if(distances[i] < DISTANCE_THRESHOLD_COUNT)
        {
            counter++;
        }
    }
    // cout << counter << " POINTS NEARBY CLUSTER" << endl;
    
    // Debug drawing points
    // Color c;
    // c.r = 255; c.g = 255; c.b = 255;
    // paintPoints(fullCloudNoCluster, points, c);
    // *fullCloudNoCluster += *(cluster.cloud);
    // viewer->updatePointCloud(fullCloudNoCluster, "kinectcloud");

    return counter < COUNT_THRESHOLD;

}



CloudInfo chooseBestCluster(std::vector<CloudInfo> cluster_clouds, CloudPtr fullPointCloud)
{


    Index idx = 0;
    cout << "Got cloudinfo: " << cluster_clouds.size() << endl;
    // Debug print

    // for(int i = 0; i < cluster_clouds.size(); i++){
    //     cout << "Cluster " << colors[i%7] << endl;
    //     float x = cluster_clouds[i].bb.maxPoint.x - cluster_clouds[i].bb.minPoint.x;
    //     float y = cluster_clouds[i].bb.maxPoint.y - cluster_clouds[i].bb.minPoint.y;
    //     float z = cluster_clouds[i].bb.maxPoint.z - cluster_clouds[i].bb.minPoint.z;
    //     cout << "X MAX POINT -- " << x << endl;
    //     cout << "Y MAX POINT -- " << y << endl;
    //     cout << "Z MAX POINT -- " << z << endl;
    //     cout << "% y/z -- " << (y/z) * 100 << "(" << (((y/z) * 100) > 50.0) << ")" << endl;
    //     cout << "Total volume -- " << (x * y * z) << "(" <<  ((x*y*z) > 0.001) << ")" << endl;
    //     cout << "-----" << endl;        
    // }


    
    if(!choose_best_cork && cluster_clouds.size() > 0){
        return cluster_clouds[idx];    
    }


    Index highest_cloud_idx = getHighestCluster(cluster_clouds);
    CloudInfo highest_cloud = cluster_clouds[highest_cloud_idx];

    double THRESHOLD_Z_DOWN = highest_cloud.bb.centroid.z() + z_threshold;
    double THRESHOLD_Z_UP = (THRESHOLD_Z_DOWN - (2*z_threshold));
    double THRESHOLD_CENTER = center_threshold;
    // TODO: Distance to center should increase as cork pieces get removed?

    for(int i = 0; i < cluster_clouds.size(); i++){
        cout << "-------- Cluster " << colors[i%7] << "----------" << endl;
       
        if((cluster_clouds[i].bb.centroid.z() <= THRESHOLD_Z_DOWN) && (cluster_clouds[i].bb.centroid.z() > THRESHOLD_Z_UP))
        {
            cout << "Cluster " << colors[i%7] << " is high enough!" << endl;
            float dist2center = sqrt((cluster_clouds[i].bb.centroid.x() * cluster_clouds[i].bb.centroid.x()) + (cluster_clouds[i].bb.centroid.y() * cluster_clouds[i].bb.centroid.y())); 
            if(dist2center <= THRESHOLD_CENTER)
            {
                cout << "Cluster " << colors[i%7] << " is close to center!" << endl;
                if(isThereSpace(cluster_clouds[i], fullPointCloud))
                {
                    cout << "There is space around " << colors[i%7] << endl;
                    cout << "Cluster " << colors[i%7] << " is the chosen one!" << endl;
                    cout << "========================================" << endl;
                    return cluster_clouds[i];
                }
            }
        }    
    }

    cout << "No 100% choice was made. Choosing the first one of the bunch." << endl;
    return cluster_clouds[idx];
}


void cluster_extraction (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{

    CloudPtr cloud_original (new Cloud);
    *cloud_original = *cloud_in; 

    CloudNormalPtr cloud_with_normals (new CloudNormal);
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    CECExtraction(cloud_in, cloud_out, clusters, small_clusters, large_clusters, cloud_with_normals);

    if(clusters->size() == 0){
        cout << "No clusters found. Returning!" << endl;
        return;
    }

    // Parse information on clusters and group it in a CloudInfo struct
    std::vector<CloudInfo> cloud_info_clusters = clusterIndicesToCloud(clusters, cloud_out, cloud_with_normals);
    
    /*** Pre-processing for chooseBestCluster ***/
    //    Clustering big and bad shaped clusters again
    cloud_info_clusters = segmentBigClusters(cloud_info_clusters);
    //     Grouping clusters belonging to the same cork piece
    vector<CloudInfo> cluster_clouds = joinSplittedClusters(cloud_info_clusters);
    paintClustersFull(cloud_out, cluster_clouds, small_clusters, large_clusters);
    // paintClusters(cloud_out, cluster_clouds);
    
    if(cluster_clouds.size() == 0){
        cout << "No clusters found. Returning!" << endl;
        return;
    }

    CloudInfo cloud_cluster = chooseBestCluster(cluster_clouds, cloud_out);

    broadcastCorkTransform(&(cloud_cluster.bb));
    drawBoundingBox(&(cloud_cluster.bb), "cork_piece");

    CloudPtr cloud_cluster_full_res = cropBoundingBox(cloud_original, cloud_cluster.bb);
    sensor_msgs::PointCloud2 published_cork_pcd;
    pcl::toROSMsg(*cloud_cluster_full_res, published_cork_pcd);
    published_cork_pcd.header.frame_id = "camera_depth_optical_frame";
    cork_cloud.publish(published_cork_pcd); 
}




void parameterConfigure(cork_iris::PCLCorkConfig &config, uint32_t level) 
{
    // General params
    display_type = config.type;
    live = config.live;
    remove_stat_outliers = config.remove_outliers;
    smooth_cloud = config.smooth_cloud;
    choose_best_cork = config.choose_best_cork;
    add_planning_scene_cork = config.add_planning_scene_cork;

    roller = config.roller;
    roller_height_value = config.roller_height_value;
    roller_height_angle = config.roller_height_angle;
    roller_width_value = config.roller_width_value;

    cecparams.radius_search = config.radius_search;

    // Clustering params
    cecparams.leaf_size = config.leaf_size;
    cecparams.cluster_tolerance = config.cluster_tolerance;
    cecparams.min_cluster_size = config.min_cluster_size;
    cecparams.max_cluster_size = config.max_cluster_size;
    cecparams.normal_diff = config.normal_diff;
    cecparams.squared_dist = config.squared_dist;
    cecparams.curv = config.curvature;

    // Statistical outliers params
    meanK = config.mean_k;


    // Best cork algorithm params
    z_threshold = config.z_threshold;
    center_threshold = config.center_threshold;
    space_distance_threshold = config.space_distance_threshold;
    space_count_points_threshold = config.space_count_points_threshold;
    space_k_neighbors = config.space_k_neighbors;
    bad_shape_percentage_threshold = config.bad_shape_percentage_threshold;
    volume_threshold = config.volume_threshold;
    splitted_cork_distance_threshold = config.splitted_cork_distance_threshold;
    splitted_cork_normal_threshold = config.splitted_cork_normal_threshold;

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
                PCLFunctions::filterRollerBox(cloud, cork_pieces, roller_width_value, roller_height_value, roller_height_angle);
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
                    PCLFunctions::remove_outliers(cork_pieces, cork_pieces, meanK);
                }
                if (smooth_cloud) // Cloud smoothing
                {
                    PCLFunctions::cloud_smoothing(cork_pieces, cork_pieces);
                }
                
                if (display_type == 2) // Surface normals color
                {
                    PCLFunctions::surface_normals(cork_pieces, cork_pieces, cecparams.radius_search);
                }
                else if(display_type == 3) // Surface curvature color
                {
                    PCLFunctions::surface_curvatures(cork_pieces, cork_pieces, cecparams.radius_search);
                }
                else if (display_type == 4) // Clustering extraction
                {
                    cluster_extraction(cork_pieces, cork_pieces);
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
    ros::init (argc, argv, "pcl_cork");
    ros::NodeHandle n;

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
    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud);

    // Create a ROS publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2> ("/cork_iris/processed_pointcloud", 1);
    point_pub = n.advertise<geometry_msgs::PoseStamped> ("/cork_iris/cork_piece", 1);
    cork_cloud = n.advertise<sensor_msgs::PointCloud2> ("/cork_iris/cork_piece_cloud", 1);


    planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
//   ros::WallDuration sleep_t(0.5);
//   while (planning_scene_diff_publisher.getNumSubscribers() < 1)
//   {
//     sleep_t.sleep();
//   }
    ros::Rate loop_rate(10);

    // Spin
    ros::spin ();
}