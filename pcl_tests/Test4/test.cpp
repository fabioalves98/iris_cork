
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
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

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    char temp[100];
    std::string ye = getcwd(temp, sizeof(temp)) ? std::string( temp ) : std::string("");

    // pcl::io::savePCDFile("./big_segment.pcd", *(clusters[i].cloud), true);
    std::cout << "current  dir: " << ye << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("t_shape.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file big_segmented.pcd \n");
            std::cout << "eheh: " << std::endl;

        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from big_segmented.pcd with the following fields: "
        << std::endl;


    

    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud);



    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

    kdtree.setInputCloud (cloud);

    pcl::PointXYZRGB searchPoint;

    int K = 30;
    std::vector<int> adjacent_idx(K);
    std::vector<float> adjacent_distance(K);

    std::vector<int> searched(cloud->size());
    // searched.push_back(0);
    std::vector<int> to_search(cloud->size());
    std::iota(to_search.begin(), to_search.end(), 0);
    // for(int i = 0; i < to_search.size(); i++){
    //     std::cout << to_search[i] << std::endl; 

    // }
    // std::cin;
    to_search.erase(std::remove(to_search.begin(), to_search.end(), 0), to_search.end());


    // std::vector<int> current_dir = {0,0,0};
    

    std::vector<double> last_dir = {0.0,0.0,0.0};
    std::vector<int> currently_growing = {-1, -1, -1};// Keeps track of last three iterations.
    int grow_track = 2; // 2 porque estamos a procura do comprimento. isto pode ser um problema? talvez mas parece que o comprimento esta sempre ok


    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    int cluster_idx = 0;

    while(!viewer->wasStopped())
    {
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

        for(int idx = 0; idx < cloud->points.size(); idx++){
            if(!std::count(searched.begin(), searched.end(), idx)){
                searchPoint = cloud->at(idx);
                std::cout << "Search point(" << idx << "): " << searchPoint << std::endl;
            }else{
                // std::cout << "Point search ignored(" << idx << ")" << std::endl;
                continue;
            }
            adjacent_idx.clear();
            adjacent_distance.clear();

            if (kdtree.nearestKSearch(searchPoint, K, adjacent_idx, adjacent_distance) > 0){
                
                
                for (std::size_t i = 0; i < adjacent_idx.size (); ++i){
                    // std::cout << "POINT OF SEARCH: " << pointIdxNKNSearch[i] << std::endl;
                    // std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x 
                    //             << " " << (*cloud)[ pointIdxNKNSearch[i] ].y 
                    //             << " " << (*cloud)[ pointIdxNKNSearch[i] ].z 
                    //             << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
                    if(cluster_idx == 0){
                        (*cloud)[adjacent_idx[i]].r = 255;
                        (*cloud)[adjacent_idx[i]].g = 255;
                        (*cloud)[adjacent_idx[i]].b = 0;
                        subcloud->points.push_back((*cloud)[ adjacent_idx[i]]);
                    }else{
                        (*cloud)[adjacent_idx[i]].r = 0;
                        (*cloud)[adjacent_idx[i]].g = 255;
                        (*cloud)[adjacent_idx[i]].b = 255;    
                        subcloud2->points.push_back((*cloud)[ adjacent_idx[i]]);
                    }
                    searched.push_back(adjacent_idx[i]);
                    to_search.erase(std::remove(to_search.begin(), to_search.end(), adjacent_idx[i]), to_search.end());
                    // }else{
                    //     (*cloud)[adjacent_idx[i]].r = 0;
                    //     (*cloud)[adjacent_idx[i]].g = 255;
                    //     (*cloud)[adjacent_idx[i]].b = 255;    
                    //     subcloud2->points.push_back((*cloud)[ adjacent_idx[i]]);

                    // }

                }

        }

            BoundingBox bb;
            if(cluster_idx == 0) bb = computeCloudBoundingBox(subcloud);
            else bb = computeCloudBoundingBox(subcloud2);
            double compr = (bb.maxPoint.z - bb.minPoint.z) ;
            double largura = (bb.maxPoint.y - bb.minPoint.y) ;
            double altura = (bb.maxPoint.x - bb.minPoint.x) ;
            std::vector<double> dims = {largura - last_dir[1], compr- last_dir[2]}; // ignoramos a altura. talvez possa ser um problema, mas parece que altura esta sempre certa
            int growing_side = (std::max_element(dims.begin(), dims.end()) - dims.begin()) + 1;

            std::cout << "Currently tracking side: " << grow_track << std::endl;
            std::cout << "Growing side: " << growing_side << std::endl;
            currently_growing.erase(currently_growing.begin());            
            currently_growing.push_back(growing_side);
            std::cout << "growing queue: " << currently_growing[0] << " " << currently_growing[1] << " " << currently_growing[2] << std::endl;
            
            // Just started
            // if(grow_track == -1){
            //     // if(std::equal(currently_growing.begin() + 1, currently_growing.end(), currently_growing.begin())){
            //         grow_track = currently_growing[currently_growing.size() - 1];
            
                // }
            // }else{
                // if(std::equal(currently_growing.begin() + 1, currently_growing.end(), currently_growing.begin()) && currently_growing[0] != grow_track){
        
                if(currently_growing[2] != grow_track && currently_growing[1] != grow_track && currently_growing[1] != -1){
                    std::cout << "COUNT: "<<  std::count(currently_growing.begin(), currently_growing.end(), grow_track) << std::endl;
                // if(currently_growing[2] != grow_track && currently_growing[1] != grow_track){
                    std::cout << " CHANGED DIRECTION! STOP CLUSTER!" << std::endl;
                    // grow_track = -1;
                    currently_growing[0] = currently_growing[1] = currently_growing[2] = -1;
                    cluster_idx++;
                    if(cluster_idx == 2) break;
                
                }    
            // }

            std::cout << "last dir 2: " << last_dir[2] << std::endl;
            std::cout << "last dir 1: " << last_dir[1] << std::endl;
            std::cout << "last dir 0: " << last_dir[0] << std::endl;

            std::cout << "diff comprimento: " << dims[2]<< std::endl;
            std::cout << "diff largura: " << dims[1] << std::endl;
            std::cout << "diff altura: " << dims[2] << std::endl;

            if(growing_side == 2){
                std::cout << "comprimento A AUMENTAR" << std::endl;
            }
            if(growing_side == 1){
                std::cout << "largura A AUMENTAR" << std::endl;
            }
            if(growing_side == 0){
                std::cout << "altura A AUMENTAR" << std::endl;
            }

            last_dir.at(0) = altura;
            last_dir.at(1) = largura;
            last_dir.at(2) = compr;

            viewer->removeShape("bb");
            viewer->addCube(bb.position, bb.orientation, bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z, "bb", 0);  
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "bb");             
            viewer->setRepresentationToWireframeForAllActors(); 

            viewer->updatePointCloud(cloud, "kinectcloud");

            usleep(500000);

            viewer->spinOnce();
            std::cout << "------: "<< std::endl;

            // std::cout << idx << std::endl;
            // searchPoint = cloud->at(idx);
            // std::cout << searchPoint << std::endl;

        }

        while(!viewer->wasStopped())
            viewer->spinOnce();
// 
        // setViewerPointcloud(subcloud);

        // PointCloud<PointXYZ>::Ptr cloud_xyz;
        // copyPointCloud(*cloud_xyz, *cloud);
            }
    // std::cout << "hiii" << std::endl;
    return (0);
}
