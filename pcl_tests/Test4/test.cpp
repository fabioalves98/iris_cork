
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/segmentation/conditional_euclidean_clustering.h>
// #include <pcl/segmentation/region_growing.h> // testing
// #include <pcl/segmentation/supervoxel_clustering.h> // testing
// #include <pcl/segmentation/lccp_segmentation.h> // testing
// #include <pcl/segmentation/cpc_seCgmentation.h> // testing
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <unistd.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
// #include <pcl/conversions.h>

pcl::visualization::PCLVisualizer::Ptr viewer;
using namespace pcl;

struct BoundingBox{
    Eigen::Quaternionf orientation;
    Eigen::Vector3f position;
    pcl::PointXYZRGB minPoint, maxPoint;
    Eigen::Vector4f centroid;
};

pcl::visualization::PCLVisualizer::Ptr normalVis(std::string name)
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

void  smoothCloud(pcl::PointCloud<PointXYZRGB>::Ptr in, pcl::PointCloud<PointXYZRGB>::Ptr out){
    // Create a KD-Tree
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointXYZRGB, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (in);
    mls.setPolynomialOrder (3);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (mls_points);
    copyPointCloud(mls_points, *out);


}

void remove_outliers(pcl::PointCloud<PointXYZRGB>::Ptr in, pcl::PointCloud<PointXYZRGB>::Ptr out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (in);
    sor.setMeanK(2);
    sor.setStddevMulThresh (1.0);
    sor.setKeepOrganized(true);
    sor.filter (*out);
}


BoundingBox computeCloudBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
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
void shapeBasedSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::IndicesClustersPtr clusters_out)
{

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (cloud_in);
    pcl::PointXYZRGB searchPoint;

    int K = 45;
    std::vector<int> adjacent_idx(K);
    std::vector<float> adjacent_distance(K);
    std::set<int> searched;
    

    std::vector<double> last_dir = {0.0,0.0,0.0};
    std::vector<int> currently_growing = {-1, -1, -1};// Keeps track of last three iterations.
    int grow_track = 2; // 2 porque estamos a procura do comprimento. isto pode ser um problema? talvez mas parece que o comprimento esta sempre ok
    int cluster_idx = 0;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndicesPtr subcloud_indices (new pcl::PointIndices);

  
    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int idx = 0; idx < cloud_in->points.size(); idx++){
        std::cout <<" IDX: " << idx << std::endl;
        // if(!std::count(searched.begin(), searched.end(), idx)){
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
                // std::cout << "POINT OF SEARCH: " << pointIdxNKNSearch[i] << std::endl;
                // std::cout << "    "  <<   (*cloud_in)[ pointIdxNKNSearch[i] ].x 
                //      for       << " " << (*cloud_in)[ pointIdxNKNSearch[i] ].y 
                //             << " " << (*cloud_in)[ pointIdxNKNSearch[i] ].z 
                //             << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
                if(cluster_idx == 0){
                    (*cloud_in)[adjacent_idx[i]].r = 255;
                    (*cloud_in)[adjacent_idx[i]].g = 255;
                    (*cloud_in)[adjacent_idx[i]].b = 0;
                }else if(cluster_idx == 1){
                (*cloud_in)[adjacent_idx[i]].r = 255;
                (*cloud_in)[adjacent_idx[i]].g = 0;
                (*cloud_in)[adjacent_idx[i]].b = 255;    
                }else{
                (*cloud_in)[adjacent_idx[i]].r = 0;
                    (*cloud_in)[adjacent_idx[i]].g = 255;
                    (*cloud_in)[adjacent_idx[i]].b = 255;   
                }
                subcloud->points.push_back((*cloud_in)[adjacent_idx[i]]);
                // ALTERED. CHANGE IN IRIS_CORK
                if(std::find(subcloud_indices->indices.begin(), subcloud_indices->indices.end(), adjacent_idx[i]) == subcloud_indices->indices.end()){
                    subcloud_indices->indices.push_back(adjacent_idx[i]);

                }
                // }
                searched.insert(adjacent_idx[i]);
            }
        }

  

        BoundingBox bb = computeCloudBoundingBox(subcloud);
        double compr = (bb.maxPoint.z - bb.minPoint.z) ;
        double largura = (bb.maxPoint.y - bb.minPoint.y) ;
        double altura = (bb.maxPoint.x - bb.minPoint.x) ;
        std::vector<double> dims = {abs(largura - last_dir[1]), abs(compr- last_dir[2])}; // ignoramos a altura. talvez possa ser um problema, mas parece que altura esta sempre certa
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

        // if((currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[0] != grow_track && currently_growing[1] != -1) || (searched.size() == cloud_in->size())){
        if((currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[0] != -1) || (searched.size() == cloud_in->size())){
            // std::cout << "COUNT: "<<  std::count(currently_growing.begin(), currently_growing.end(), grow_track) << std::endl;
            std::cout << " CHANGED DIRECTION! STOP CLUSTER!" << std::endl;
            std::cout << " searched == cloudin " <<  (searched.size() == cloud_in->size()) << std::endl;
            std::cout << " dims0 > dims1 " <<  (dims[0] > 10*dims[1]) << std::endl;
            // grow_track = -1;
            currently_growing[0] = currently_growing[1] = currently_growing[2] = -1;
            // pcl::PointIndices indices (new pcl::PointIndices);
            // indices.indices = subcloud_indices.indices;
            clusters_out->push_back(*subcloud_indices);
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
        std::cout << "-------" << std::endl;
        usleep(100000);

        viewer->removeShape("cork_piece");
        viewer->addCube(bb.position, bb.orientation, bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z, "cork_piece", 0);  
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cork_piece");             
        viewer->setRepresentationToWireframeForAllActors(); 
        viewer->updatePointCloud(cloud_in, "kinectcloud");
        viewer->spinOnce();

    }


}

void shapeBasedSegmentationv2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::IndicesClustersPtr clusters_out)
{

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (cloud_in);
    pcl::PointXYZRGB searchPoint;

    int K = 100;
    std::vector<int> adjacent_idx(K);
    std::vector<float> adjacent_distance(K);
    std::set<int> searched;
    std::set<int> to_search;
    // std::iota (std::begin(to_search), std::end(to_search), 0);
    // for(int i = 0; i < cloud_in->size(); i++){
    //     to_search.insert(i);
    // }

    float radius = 0.1;

    std::vector<double> last_dir = {0.0,0.0,0.0};
    std::vector<int> currently_growing = {-1, -1, -1};// Keeps track of last three iterations.
    int grow_track = 2; // 2 porque estamos a procura do comprimento. isto pode ser um problema? talvez mas parece que o comprimento esta sempre ok
    int cluster_idx = 0;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndicesPtr subcloud_indices (new pcl::PointIndices);

  
    int idx = 0;

    while(true){
        searchPoint = cloud_in->points.at(idx);
        if(kdtree.radiusSearch(searchPoint, radius, adjacent_idx, adjacent_distance) > 0){
            for(std::size_t i = 0; i < adjacent_idx.size (); ++i){
                // std::cout << "POINT OF SEARCH: " << pointIdxNKNSearch[i] << std::endl;
                // std::cout << "    "  <<   (*cloud_in)[ pointIdxNKNSearch[i] ].x 
                //      for       << " " << (*cloud_in)[ pointIdxNKNSearch[i] ].y 
                //             << " " << (*cloud_in)[ pointIdxNKNSearch[i] ].z 
                //             << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
                std::cout << " DISTANCE: " << adjacent_distance[i] << std::endl;
                (*cloud_in)[idx].r = 255;
                (*cloud_in)[idx].g = 0;
                (*cloud_in)[idx].b = 0;
                if(adjacent_distance[i] > 0.003){
                    
                    if(cluster_idx == 0){
                        (*cloud_in)[adjacent_idx[i]].r = 255;
                        (*cloud_in)[adjacent_idx[i]].g = 255;
                        (*cloud_in)[adjacent_idx[i]].b = 0;
                    }else if(cluster_idx == 1){
                        (*cloud_in)[adjacent_idx[i]].r = 255;
                        (*cloud_in)[adjacent_idx[i]].g = 0;
                        (*cloud_in)[adjacent_idx[i]].b = 255;    
                    }else{
                        (*cloud_in)[adjacent_idx[i]].r = 0;
                        (*cloud_in)[adjacent_idx[i]].g = 255;
                        (*cloud_in)[adjacent_idx[i]].b = 255;   
                    }
                }else{
                    (*cloud_in)[adjacent_idx[i]].r = 0;
                    (*cloud_in)[adjacent_idx[i]].g = 255;
                    (*cloud_in)[adjacent_idx[i]].b = 0;    
                }

                if(searched.find(adjacent_idx[i]) == searched.end()){

                    subcloud->points.push_back((*cloud_in)[adjacent_idx[i]]);
                    if(std::find(subcloud_indices->indices.begin(), subcloud_indices->indices.end(), adjacent_idx[i]) == subcloud_indices->indices.end()){
                        subcloud_indices->indices.push_back(adjacent_idx[i]);
                    }
                    // }
                    searched.insert(adjacent_idx[i]);
                    to_search.insert(adjacent_idx[i]);
                    // if((to_search.find(adjacent_idx[i]) != to_search.end())){
                    //     to_search.erase(to_search.find(adjacent_idx[i]));
                }
                
            }    
        }

        std::set<int>::iterator it = to_search.begin();
        float min_dist = 100;
        idx = -1;
        while(it != to_search.end()){
            float dist = pcl::squaredEuclideanDistance(searchPoint, cloud_in->points.at(*it)); 
            if (dist > 0){
                idx = (*it);
            } 
            // if(dist < min_dist){
            //     min_dist = dist;
            //     if(min_dist < 0.005){ // Isto evita que ele dps va para o outro lado e cenas desse genero.
            //         idx = (*it);
            //     }
            //     //     std::cout << "current subcloudsize: " << subcloud->size() << std::endl;
            //     //     if(subcloud->size() < 500){
            //     //         next_idx = (*it);
            //     //     }
            //     // }

            // }

            it++;
        }
        std::cout << "Reached the end, idx: " << idx << std::endl; 

        if(idx == -1){
            break;
        }
        
        BoundingBox bb = computeCloudBoundingBox(subcloud);
        double compr = (bb.maxPoint.z - bb.minPoint.z) ;
        double largura = (bb.maxPoint.y - bb.minPoint.y) ;
        double altura = (bb.maxPoint.x - bb.minPoint.x) ;
        std::vector<double> dims = {abs(largura - last_dir[1]), abs(compr- last_dir[2])}; // ignoramos a altura. talvez possa ser um problema, mas parece que altura esta sempre certa
        int growing_side = (std::max_element(dims.begin(), dims.end()) - dims.begin()) + 1;

        std::cout << "Currently tracking side: " << grow_track << std::endl;
        std::cout << "Growing side: " << growing_side << std::endl;
        currently_growing.erase(currently_growing.begin());            
        currently_growing.push_back(growing_side);
        std::cout << "growing queue: " << currently_growing[0] << " " << currently_growing[1] << " " << currently_growing[2] << std::endl;
        std::cout << "dims : " << dims[0] << " " << dims[1]      << std::endl;
        
 
        // if((currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[0] != grow_track && currently_growing[1] != -1) || (searched.size() == cloud_in->size())){
        if((currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[0] != -1) || (searched.size() == cloud_in->size())){
            // std::cout << "COUNT: "<<  std::count(currently_growing.begin(), currently_growing.end(), grow_track) << std::endl;
            std::cout << " CHANGED DIRECTION! STOP CLUSTER!" << std::endl;
            std::cout << " searched == cloudin " <<  (searched.size() == cloud_in->size()) << std::endl;
            std::cout << " dims0 > dims1 " <<  (dims[0] > 10*dims[1]) << std::endl;
            currently_growing[0] = currently_growing[1] = currently_growing[2] = -1;
            clusters_out->push_back(*subcloud_indices);
            subcloud_indices->indices.clear();
            subcloud->points.clear();
            // next_idx = idx;
            cluster_idx++;
        
        }    
     
        last_dir.at(0) = altura;
        last_dir.at(1) = largura;
        last_dir.at(2) = compr;
        std::cout << "-------" << std::endl;
        usleep(1000000);

        // idx = next_idx;

        // if(cluster_idx == 0){

        viewer->removeShape("cork_piece");
        viewer->addCube(bb.position, bb.orientation, bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z, "cork_piece", 0);  
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cork_piece");             
        viewer->setRepresentationToWireframeForAllActors(); 
        viewer->updatePointCloud(cloud_in, "kinectcloud");
        viewer->spinOnce();
        // }else{
        //     viewer->removeShape("cork_piece2");
        //     viewer->addCube(bb.position, bb.orientation, bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z, "cork_piece2", 0);  
        //     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cork_piece2");             
        //     viewer->setRepresentationToWireframeForAllActors(); 
        //     viewer->updatePointCloud(cloud_in, "kinectcloud");
        //     viewer->spinOnce();
        // }


    }

   

}

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
    char temp[100];
    std::string ye = getcwd(temp, sizeof(temp)) ? std::string( temp ) : std::string("");
    std::cout << cloud_in->isOrganized() << std::endl;

    // pcl::io::savePCDFile("./big_segment.pcd", *(clusters[i].cloud), true);
    std::cout << "current  dir: " << ye << std::endl;
    std::cout << "Insert cloud filename: ";

    std::string file;
    cin >> file;
    if(!file.compare("+") ){
        file = "t_shape.pcd";
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file, *cloud_in) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file big_segmented.pcd \n");
            std::cout << "eheh: " << std::endl;

        return (-1);
    }
    // std::cout << "Loaded "
    //     << cloud->width * cloud->height
    //     << " data points from big_segmented.pcd with the following fields: "
    //     << std::endl;


    

    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud_in);

    // smoothCloud(cloud_in, cloud_in);
    pcl::IndicesClustersPtr out_indices (new pcl::IndicesClusters);
    shapeBasedSegmentationv2(cloud_in, out_indices);
    // pcl::PointIndices::Ptr cloud_indices (new pcl::PointIndices);
    // cloud_indices->indices = (*out_indices)[0].indices;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

    // // copyPointCloud(*cloud_in, cloud_indices, *cloud_out);
    // pcl::ExtractIndices<pcl::PointXYZRGB> filter;
    // filter.setInputCloud (cloud_in);
    // filter.setIndices (cloud_indices);
    // filter.setNegative (false);
    // // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
    // filter.setKeepOrganized (true);
    // filter.filter (*cloud_out);

    // std::cout << *cloud_indices << std::endl;
    // std::cout << cloud_in->points.size() << std::endl;
    // std::cout << cloud_out->isOrganized() << std::endl;
    // std::cout << cloud_out->width << std::endl;
    // std::cout << cloud_out->height << std::endl;
    // remove_outliers(cloud_out, cloud_out);
    // BoundingBox bb = computeCloudBoundingBox(cloud_out);
    // viewer->removeShape("cork_piece2");
    // viewer->addCube(bb.position, bb.orientation, bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z, "cork_piece2", 0);  
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cork_piece2");             
    // viewer->setRepresentationToWireframeForAllActors(); 
    // viewer->updatePointCloud(cloud_in, "kinectcloud");
    // viewer->spinOnce();

    // for(int i = 0; i < out_indices->size(); i++){


    //     for (int j = 0; j < (*out_indices)[i].indices.size (); ++j){
    //         if(i == 0){
    //             (*cloud_in)[(*out_indices)[i].indices[j]].r = 255;
    //             (*cloud_in)[(*out_indices)[i].indices[j]].g = 255;
    //             (*cloud_in)[(*out_indices)[i].indices[j]].b = 0;  
    //         }else if(i == 1){
    //             (*cloud_in)[(*out_indices)[i].indices[j]].r = 255;
    //             (*cloud_in)[(*out_indices)[i].indices[j]].g = 0;
    //             (*cloud_in)[(*out_indices)[i].indices[j]].b = 255;    
    //         }else{
    //            (*cloud_in)[(*out_indices)[i].indices[j]].r = 0;
    //             (*cloud_in)[(*out_indices)[i].indices[j]].g = 255;
    //             (*cloud_in)[(*out_indices)[i].indices[j]].b = 255;   
    //         }
    //     }
    // }

    viewer->updatePointCloud(cloud_in, "kinectcloud");
    while(!viewer->wasStopped())
        viewer->spinOnce();

   
    return (0);
}
