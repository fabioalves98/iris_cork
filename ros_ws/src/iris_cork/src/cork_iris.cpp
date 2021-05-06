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
        CloudInfo cloud_info;
        // pcl::ExtractIndices<pcl::PointXYZRGB> filter;
        // cout << " original cloud organized? " << original_cloud->isOrganized() << endl;
        // filter.setInputCloud (original_cloud);
        // filter.setIndices (cloud_indices);
        // filter.setNegative (false);
        // // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
        // filter.setKeepOrganized (true);
        // filter.filter (*(cloud_info.cloud));
        // cout << "after original cloud organized? " << cloud_info.cloud->isOrganized() << endl;


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
            cout << "Cluster " << colors[i%7] << " is either too big or bad shaped. Segmenting again" << endl;
            cout << "Cluster bad shaped: " << isClusterBadShaped(clusters[i].bb) << endl;
            cout << "Cluster too big: " << isClusterTooBig(clusters[i].bb) << endl;

            IdxClustersPtr cluster_indices (new IdxClusters);
            CorkIris::shapeBasedSegmentation(clusters[i].cloud, cluster_indices);
            std::vector<CloudInfo> cloud_info_clusters = clusterIndicesToCloud(cluster_indices, cloud_in, cloud_with_normals);

            for(int i = 0; i < cloud_info_clusters.size(); i++){
                new_vec.push_back(cloud_info_clusters[i]);
                cout << "Segmented a new big cluster. Adding to array."<< endl;

            }
            
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


void CorkIris::shapeBasedSegmentation(CloudPtr cloud_in, IdxClustersPtr clusters_indices){


    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (cloud_in);
    pcl::PointXYZRGB searchPoint;

    int K = test3;
    std::vector<int> adjacent_idx(K);
    std::vector<float> adjacent_distance(K);
    std::set<int> searched;
    

    std::vector<double> last_dir = {0.0,0.0,0.0};
    std::vector<int> currently_growing = {-1, -1, -1};// Keeps track of last three iterations.
    int grow_track = 2; // 2 porque estamos a procura do comprimento. isto pode ser um problema? talvez mas parece que o comprimento esta sempre ok
    int cluster_idx = 0;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndicesPtr subcloud_indices (new pcl::PointIndices);


    for(int idx = 0; idx < cloud_in->points.size(); idx++){
        // std::cout <<" IDX: " << idx << std::endl;
        if(searched.find(idx) == searched.end()){
            searchPoint = cloud_in->at(idx);
            std::cout << "Search point(" << idx << "): " << searchPoint << std::endl;
        }else{
            // std::cout << "Point search ignored(" << idx << ")" << std::endl;
            continue;
        }
        adjacent_idx.clear();
        adjacent_distance.clear();

        if (kdtree.nearestKSearch(searchPoint, K, adjacent_idx, adjacent_distance) > 0){
            for(std::size_t i = 0; i < adjacent_idx.size (); ++i){
                cout << "DISTANCE: " << adjacent_distance[i] << endl;
                if(adjacent_distance[i] < test2){
                    subcloud->points.push_back((*cloud_in)[adjacent_idx[i]]);
                
                    if(std::find(subcloud_indices->indices.begin(), subcloud_indices->indices.end(), adjacent_idx[i]) == subcloud_indices->indices.end()){
                        subcloud_indices->indices.push_back(adjacent_idx[i]);
                    }
                    
                    // subcloud_indices->indices.push_back(adjacent_idx[i]);
                    
                    searched.insert(adjacent_idx[i]);
                }
            }else{
                cout <<" Point too far not adding!" << endl;
            }

        }

  

        BoundingBox bb = PCLFunctions::computeCloudBoundingBox(subcloud);
        double compr = (bb.maxPoint.z - bb.minPoint.z) ;
        double largura = (bb.maxPoint.y - bb.minPoint.y) ;
        double altura = (bb.maxPoint.x - bb.minPoint.x) ;
        std::vector<double> dims = {abs(largura - last_dir[1]), abs(compr - last_dir[2])}; // ignoramos a altura. talvez possa ser um problema, mas parece que altura esta sempre certa
        int growing_side = (std::max_element(dims.begin(), dims.end()) - dims.begin()) + 1;

        std::cout << "Currently tracking side: " << grow_track << std::endl;
        std::cout << "Growing side: " << growing_side << std::endl;
        currently_growing.erase(currently_growing.begin());            
        currently_growing.push_back(growing_side);
        std::cout << "growing queue: " << currently_growing[0] << " " << currently_growing[1] << " " << currently_growing[2] << std::endl;
        std::cout << "dims : " << dims[0] << " " << dims[1]      << std::endl;
        
        // if(grow_track == -1){
        //     if(currently_growing[2] == currently_growing[1]){
        //         grow_track = growing_side;
        //     }
        // }

        bool cond;

        if(test1 > 0.5){
            cond = (currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[0] != -1) || (searched.size() == cloud_in->size());
        }else{
            cond = (currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[0] != grow_track && currently_growing[1] != -1) || (searched.size() == cloud_in->size());
        }

        // if((currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[0] != grow_track && currently_growing[1] != -1) || (searched.size() == cloud_in->size())){
        // if((currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[0] != -1) || (searched.size() == cloud_in->size())){
        if(cond){
            // std::cout << "COUNT: "<<  std::count(currently_growing.begin(), currently_growing.end(), grow_track) << std::endl;
            std::cout << " CHANGED DIRECTION! STOP CLUSTER!" << std::endl;
            std::cout << " searched == cloudin " <<  (searched.size() == cloud_in->size()) << std::endl;
            std::cout << " dims0 > dims1 " <<  (dims[0] > 10*dims[1]) << std::endl;
            // grow_track = -1;
            currently_growing[0] = currently_growing[1] = currently_growing[2] = -1;
            // pcl::PointIndices indices (new pcl::PointIndices);
            // indices.indices = subcloud_indices.indices;
            clusters_indices->push_back(*subcloud_indices);
            // std::cout << *subcloud_indices << std::endl;
            subcloud_indices->indices.clear();
            // std::cout<< "AFTER CLEAR:" << *subcloud_indices << std::endl;
            // std::cout<< "AFTER CLEAR:" << clusters_out[0] << std::endl;
            subcloud->points.clear();
            // grow_track = -1;
            // if(searched.size() < cloud_in->size()){
            //     // searched.erase(--searched.end());
            //     idx = *searched.rbegin();
            //     std::cout << "new idx: " << idx << std::endl;
            // }
            cluster_idx++;
        
        }    
        last_dir.at(0) = altura;
        last_dir.at(1) = largura;
        last_dir.at(2) = compr;
        // std::cout << "-------" << std::endl;
        // usleep(100000);
    }
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

    // DEBUG SAVE CLUSTER CLOUDS TO INDIVIDUAL FILE

    for(int i = 0; i < cluster_clouds.size(); i++){
        pcl::io::savePCDFile("./individual_cork_strip" + std::to_string(i) + ".pcd", *(cluster_clouds[i].cloud), true);
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


