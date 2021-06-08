#include "cork_iris.h"
#include <pcl/io/pcd_io.h> // testing
double test1, test2;
int test3, test4;

void CorkIris::updateParams(iris_cork::PCLCorkConfig &config)
{
    choose_best_cork = config.choose_best_cork;

    chosen_cork_strip = config.selected_cork_strip;

    // Best cork algorithm params
    z_threshold = config.z_threshold;
    center_threshold = config.center_threshold;
    space_distance_threshold = config.space_distance_threshold;
    space_count_points_threshold = config.space_count_points_threshold;
    space_k_neighbors = config.space_k_neighbors;
    bad_shape_width_threshold = config.bad_shape_width_threshold;
    volume_threshold = config.volume_threshold;
    splitted_cork_distance_threshold = config.splitted_cork_distance_threshold;
    splitted_cork_normal_threshold = config.splitted_cork_normal_threshold;

    height_weight = config.height_weight;
    distance_weight = config.distance_weight;
    size_weight = config.size_weight;

    test1 = config.test_param1;
    test2 = config.test_param2;
    test3 = config.test_param3;
    test4 = config.test_param4;
}

CloudInfo CorkIris::clusterExtraction(CloudPtr cloud_in, CloudPtr cloud_out)
{
    // Perform Conditional Euclidean Clustering on the main cloud
    CloudNormalPtr cloud_with_normals (new CloudNormal);
    IdxClustersPtr clusters (new IdxClusters), small_clusters (new IdxClusters), large_clusters (new IdxClusters);
    PCLFunctions::cec_extraction(cloud_in, cloud_out, clusters, small_clusters, large_clusters, cloud_with_normals);
    cout << "after cec extraction" << cloud_out->isOrganized() << endl;

    if(clusters->size() == 0){
        std::cout << "No clusters found. Returning!" << std::endl;
        CloudInfo empty;
        return empty;
    }

    // Parse information on clusters and group it in a CloudInfo struct
    std::vector<CloudInfo> cloud_info_clusters = clusterIndicesToCloud(clusters, cloud_out, cloud_with_normals);
    
    // Clustering big and bad shaped clusters again
    cloud_info_clusters = segmentBigClusters(cloud_out, cloud_with_normals, cloud_info_clusters);
    
    // Grouping clusters belonging to the same cork piece
    std::vector<CloudInfo> cluster_clouds = joinSplittedClusters(cloud_info_clusters);
    
    if(cluster_clouds.size() == 0){
        cout << "No clusters found. Returning!" << endl;
        CloudInfo empty;
        return empty;
    }
    
    // Choose the best cluster from the extracted and filtered ones.
    CloudInfo best_cluster = chooseBestCluster(cluster_clouds, cloud_out);
    

    // Paint Clusters for visualization before returning.
    paintClustersFull(cloud_out, cluster_clouds, small_clusters, large_clusters);
    
    return best_cluster; 
}

std::vector<CloudInfo> CorkIris::clusterIndicesToCloud(IdxClustersPtr clusters, CloudPtr original_cloud, CloudNormalPtr original_cloud_normal)
{
    std::vector<CloudInfo> cloud_clusters;
    for(int i = 0; i < clusters->size(); i++)
    {
        CloudPtr cloud_cluster (new Cloud);
        CloudNormalPtr cloud_normal_cluster (new CloudNormal);
        pcl::PointIndices::Ptr cloud_indices (new pcl::PointIndices);
        cloud_indices->indices = (*clusters)[i].indices;
        CloudInfo cloud_info;

        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
        {
            cloud_cluster->push_back(original_cloud->points[(*clusters)[i].indices[j]]);
            cloud_normal_cluster->push_back(original_cloud_normal->points[(*clusters)[i].indices[j]]);
        }

        cloud_info.cloud = cloud_cluster;
        // copyPointCloud(*original_cloud, *cloud_indices, *(cloud_info.cloud));
        cloud_info.cloudNormal = cloud_normal_cluster;
        cloud_info.bb = PCLFunctions::computeCloudBoundingBox(cloud_cluster);
        cloud_info.indices = cloud_indices;

        // Average cork distance to camera
        cloud_info.cloud_size = (*clusters)[i].indices.size();
        float distance = 0;
        for(int idx = 0; idx < cloud_info.cloud_size; idx++){
            distance += cloud_cluster->at(idx).z;
        }
        cloud_info.average_height = distance / cloud_info.cloud_size;
        cloud_info.distance_manipulator = 1.0; // TODO: Properly define this.
        
        cloud_clusters.push_back(cloud_info);
    }

    return cloud_clusters;
}



std::vector<CloudInfo> CorkIris::segmentBigClusters(CloudPtr cloud_in, CloudNormalPtr cloud_with_normals, std::vector<CloudInfo> clusters)
{
    std::vector<CloudInfo> new_vec;
    cout << "<<<<< SEGMENTING BIG/OUT OF SHAPE CLUSTERS >>>>>>" << endl;
    CloudPtr display_cloud (new Cloud);
    for(int i = 0; i < clusters.size(); i++)
    {  

        if(isClusterBadShaped(clusters[i].bb) || isClusterTooBig(clusters[i].bb))
        {
            // cout << "Cluster " << colors[i%7] << " is either too big or bad shaped. Segmenting again" << endl;
            // cout << "Cluster bad shaped: " << isClusterBadShaped(clusters[i].bb) << endl;
            // cout << "Cluster too big: " << isClusterTooBig(clusters[i].bb) << endl;

            IdxClustersPtr cluster_indices (new IdxClusters);
            std::vector<CloudInfo> cloud_info_clusters = clusterIndicesToCloud(cluster_indices, cloud_in, cloud_with_normals);

            // for(int i = 0; i < cloud_info_clusters.size(); i++){
            //     new_vec.push_back(cloud_info_clusters[i]);
            //     cout << "Segmented a new big cluster. Adding to array."<< endl;

            // }
            
           }else{
            new_vec.push_back(clusters[i]);

        }

    }
    // viewer->updatePointCloud(display_cloud, "kinectcloud");


    cout << "ENDED SEGMENTATION" << endl; 
    return new_vec;
}

/*
    Determines if the shape of the cluster is "cork" like
*/
bool CorkIris::isClusterBadShaped(BoundingBox cluster)
{
    float THRESHOLD_WIDTH = bad_shape_width_threshold;
    float largura = cluster.maxPoint.y - cluster.minPoint.y;
    cout << "largura: " << largura << endl;
    // float comprimento = cluster.maxPoint.z - cluster.minPoint.z;

    return largura > THRESHOLD_WIDTH;
}

/*
    Determines if the cluster bounding box is too big to be a "cork_piece"
*/
bool CorkIris::isClusterTooBig(BoundingBox cluster)
{
    float THRESHOLD_VOLUME = volume_threshold;
    float largura = cluster.maxPoint.y - cluster.minPoint.y;
    float comprimento = cluster.maxPoint.z - cluster.minPoint.z;
    float altura = cluster.maxPoint.x - cluster.minPoint.x;
    return (largura * comprimento * altura) > THRESHOLD_VOLUME;
}

std::vector<CloudInfo> CorkIris::joinSplittedClusters(std::vector<CloudInfo> clusters)
{
    std::vector<CloudInfo> new_vec;
    cout << "<<<<< JOINING SPLITTED CLUSTERS >>>>>>" << endl;
    if (clusters.size() <= 1)
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
    Determines if a cork_piece was splitted into two different clusters (Works best if the clusters inserted here are one after the other
    in the original cluster array, since this means they were clustered one after the other. Clusters where this didn't happend should belong
    to the same cork_piece).
*/
bool CorkIris::isSplittedCluster(CloudInfo cluster0, CloudInfo cluster1)
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

/*
    Computes an average normal associated with a cluster.
*/
Eigen::Vector3f CorkIris::getClusterAverageNormal(CloudNormalPtr cluster)
{
    Eigen::Vector3f currentNormal = cluster->points[0].getNormalVector3fMap();
    for(int i = 1; i < cluster->points.size(); i++)
    {
        currentNormal += cluster->points[i].getNormalVector3fMap();
    }

    return currentNormal / cluster->points.size();
}

CloudInfo CorkIris::joinClusters(CloudInfo cluster0, CloudInfo cluster1)
{
    cout << "Cloud b4:" << cluster0.cloud->size() << endl;
    *(cluster0.cloud) += *(cluster1.cloud);
    *(cluster0.cloudNormal) += *(cluster1.cloudNormal);
    (cluster0.indices->indices).insert(cluster0.indices->indices.end(), cluster1.indices->indices.begin(), cluster1.indices->indices.end());
    
    if(cluster1.bb.centroid.z() > cluster0.bb.centroid.z())
    {
        cluster0.bb = cluster1.bb;
    }
    cout << "Centrofd BB0: " << cluster0.bb.centroid << endl << "Centroid BB1: " << cluster1.bb.centroid << endl;
    cout << "Cloud after:" << cluster0.cloud->size() << endl;

    return cluster0;
}

void CorkIris::paintClustersFull(CloudPtr cloud_out, std::vector<CloudInfo> clusters, IdxClustersPtr small_clusters, IdxClustersPtr large_clusters)
{
    std::sort(std::begin(clusters), std::end(clusters), [](const CloudInfo &a, const CloudInfo &b){return a.score > b.score;});

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

void CorkIris::paintClusters(CloudPtr cloud_out, std::vector<CloudInfo> clusters)
{
    for (int i = 0; i < clusters.size(); ++i)
    {      

        for (int j = 0; j < clusters[i].indices->indices.size (); ++j)
        {
            std::vector<int> color = (i >= gradient_colors.size()) ? gradient_colors[gradient_colors.size()-1] : gradient_colors[i]; 
            cloud_out->points[clusters[i].indices->indices[j]].r = color[0];
            cloud_out->points[clusters[i].indices->indices[j]].g = color[1];
            cloud_out->points[clusters[i].indices->indices[j]].b = color[2];            
        
        }
    }  
}

CloudInfo CorkIris::chooseBestCluster(std::vector<CloudInfo> cluster_clouds, CloudPtr fullPointCloud)
{
    Index idx = 0;
    cout << "Got cloudinfo: " << cluster_clouds.size() << endl;
   
    if(chosen_cork_strip > -1 && chosen_cork_strip < cluster_clouds.size()){
        return cluster_clouds[chosen_cork_strip];
    }

    if(!choose_best_cork && cluster_clouds.size() > 0){
        return cluster_clouds[idx];    
    }

    // // DEBUG SAVE CLUSTER CLOUDS TO INDIVIDUAL FILE
    // for(int i = 0; i < cluster_clouds.size(); i++){
    //     pcl::io::savePCDFile("./individual_cork_strip" + std::to_string(i) + ".pcd", *(cluster_clouds[i].cloud), true);
    // }


    Index highest_cloud_idx = getHighestCluster(cluster_clouds);
    CloudInfo highest_cloud = cluster_clouds[highest_cloud_idx];

    Index largest_cloud_idx = getBiggestCluster(cluster_clouds);
    CloudInfo largest_cloud = cluster_clouds[largest_cloud_idx];

    // Set the score for each parameter and calculate final score using the weights from dynamic reconfigure.
    for(Index idx = 0; idx < cluster_clouds.size(); idx++)
    {
        cluster_clouds[idx].height_score = 1/(cluster_clouds[idx].average_height / highest_cloud.average_height);
        cluster_clouds[idx].size_score = cluster_clouds[idx].cloud_size / largest_cloud.cloud_size;
        cluster_clouds[idx].distance_score = cluster_clouds[idx].distance_manipulator / highest_cloud.distance_manipulator;

        // TODO take into account the nr of points to grab or something related to that

        cluster_clouds[idx].score = (cluster_clouds[idx].height_score * height_weight) 
        + (cluster_clouds[idx].size_score * size_weight) 
        + (cluster_clouds[idx].distance_score * distance_weight); 

        std::cout << colors[idx%7] << " has an height of: " << cluster_clouds[idx].average_height << std::endl;
        std::cout << colors[idx%7] << " has an size of: " << cluster_clouds[idx].cloud_size << std::endl;
        std::cout << colors[idx%7] << " has an distance of: " << cluster_clouds[idx].distance_manipulator << std::endl;


        std::cout << colors[idx%7] << " has an height score of: " << cluster_clouds[idx].height_score << std::endl;
        std::cout << colors[idx%7] << " has a size score of: " << cluster_clouds[idx].size_score << std::endl;
        std::cout << colors[idx%7] << " has distance score of: " << cluster_clouds[idx].distance_score << std::endl;
        std::cout << colors[idx%7] << " has a score of: " << cluster_clouds[idx].score << std::endl;
        std::cout << "------\n" << std::endl;

    }

    idx = getBestClusterToGrab(cluster_clouds);

    return cluster_clouds[idx];
}

// Chooses the cluster with the biggest "average_height" attribute from CloudInfo
Index CorkIris::getHighestCluster(std::vector<CloudInfo> clusters)
{
    if(clusters.size() == 0) return -1;

    Index idx = 0;
    float highest = clusters[0].average_height;
    cout << "highest: " << highest << endl;
    for(int i = 1; i < clusters.size(); i++)
    {
        if(clusters[i].average_height <= highest)
        {
            highest = clusters[i].average_height;
            idx = i;
        }
    }
    return idx;
}

// Chooses the cluster with the biggest "cloud_size" attribute from CloudInfo
Index CorkIris::getBiggestCluster(std::vector<CloudInfo> clusters)
{

    if(clusters.size() == 0) return -1;

    Index idx = 0;
    float highest = clusters[0].cloud_size;
    for(int i = 1; i < clusters.size(); i++)
    {
        if(clusters[i].cloud_size > highest)
        {
            highest = clusters[i].cloud_size;
            idx = i;
        }
    }
    return idx;

}

Index CorkIris::getBestClusterToGrab(std::vector<CloudInfo> clusters)
{
    if(clusters.size() == 0) return -1;

    Index idx = 0;
    float highest = clusters[0].score;
    for(int i = 1; i < clusters.size(); i++)
    {
        if(clusters[i].score > highest)
        {
            highest = clusters[i].score;
            idx = i;
        }
    }
    return idx;

}
bool CorkIris::isThereSpace(CloudInfo cluster, CloudPtr fullCloud)
{
    CloudPtr fullCloudNoCluster = PCLFunctions::subtractCloud(fullCloud, cluster.indices);
    if (fullCloudNoCluster->size() == 0)
    {
        return true;
    }
    
    int K = space_k_neighbors;
    float DISTANCE_THRESHOLD_COUNT = space_distance_threshold;
    int COUNT_THRESHOLD = space_count_points_threshold;

    std::vector<int> points(K);
    std::vector<float> distances(K);
    PCLFunctions::getNearestNeighbors(K, cluster.bb.centroid, fullCloudNoCluster, points, distances);
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


