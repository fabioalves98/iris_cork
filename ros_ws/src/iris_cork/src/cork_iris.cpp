#include "cork_iris.h"

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
    bad_shape_percentage_threshold = config.bad_shape_percentage_threshold;
    volume_threshold = config.volume_threshold;
    splitted_cork_distance_threshold = config.splitted_cork_distance_threshold;
    splitted_cork_normal_threshold = config.splitted_cork_normal_threshold;
}

CloudInfo CorkIris::clusterExtraction(CloudPtr cloud_in, CloudPtr cloud_out)
{
    // Perform Conditional Euclidean Clustering on the main cloud
    CloudNormalPtr cloud_with_normals (new CloudNormal);
    IdxClustersPtr clusters (new IdxClusters), small_clusters (new IdxClusters), large_clusters (new IdxClusters);
    PCLFunctions::cec_extraction(cloud_in, cloud_out, clusters, small_clusters, large_clusters, cloud_with_normals);

    if(clusters->size() == 0){
        std::cout << "No clusters found. Returning!" << std::endl;
        CloudInfo empty;
        return empty;
    }

    // Parse information on clusters and group it in a CloudInfo struct
    std::vector<CloudInfo> cloud_info_clusters = clusterIndicesToCloud(clusters, cloud_out, cloud_with_normals);
    
    // Clustering big and bad shaped clusters again
    cloud_info_clusters = segmentBigClusters(cloud_info_clusters);
    
    // Grouping clusters belonging to the same cork piece
    std::vector<CloudInfo> cluster_clouds = joinSplittedClusters(cloud_info_clusters);
    
    // Paint Clusters for visualization
    paintClustersFull(cloud_out, cluster_clouds, small_clusters, large_clusters);
        
    if(cluster_clouds.size() == 0){
        cout << "No clusters found. Returning!" << endl;
        CloudInfo empty;
        return empty;
    }

    // Chooses one single cluster from all present in cloud
    return chooseBestCluster(cluster_clouds, cloud_out);
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
        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
        {
            cloud_cluster->push_back(original_cloud->points[(*clusters)[i].indices[j]]);
            cloud_normal_cluster->push_back(original_cloud_normal->points[(*clusters)[i].indices[j]]);
        }
        CloudInfo cloud_info;
        cloud_info.cloud = cloud_cluster;
        cloud_info.cloudNormal = cloud_normal_cluster;
        cloud_info.bb = PCLFunctions::computeCloudBoundingBox(cloud_cluster);
        cloud_info.indices = cloud_indices;
        cloud_clusters.push_back(cloud_info); 
    }

    return cloud_clusters;
}

std::vector<CloudInfo> CorkIris::segmentBigClusters(std::vector<CloudInfo> clusters)
{
    std::vector<CloudInfo> new_vec;
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

/*
    Determines if the shape of the cluster is "cork" like
*/
bool CorkIris::isClusterBadShaped(BoundingBox cluster)
{
    float THRESHOLD_PERCENTAGE = bad_shape_percentage_threshold;
    float largura = cluster.maxPoint.y - cluster.minPoint.y;
    float comprimento = cluster.maxPoint.z - cluster.minPoint.z;

    return ((largura/comprimento)) > THRESHOLD_PERCENTAGE;
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
        std::bitset<8> binary = std::bitset<8>(i);

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

CloudInfo CorkIris::chooseBestCluster(std::vector<CloudInfo> cluster_clouds, CloudPtr fullPointCloud)
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

    if(chosen_cork_strip > -1 && chosen_cork_strip < cluster_clouds.size()){
        return cluster_clouds[chosen_cork_strip];
    }

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

Index CorkIris::getHighestCluster(std::vector<CloudInfo> clusters)
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


