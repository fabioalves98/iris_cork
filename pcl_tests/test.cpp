
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/ml/kmeans.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>

pcl::visualization::PCLVisualizer::Ptr viewer;
using namespace pcl;

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


pcl::PointCloud<pcl::PointXYZRGB>::Ptr iterativeBoundaryRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int iterations){
    
    NormalEstimation<PointXYZRGB, Normal> ne;
	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> est;

	PointCloud<PointXYZRGB>::Ptr process_cloud (new PointCloud<PointXYZRGB>);
    copyPointCloud(*cloud_in, *process_cloud);

    std::cout << "proces cloud size: " << process_cloud->size() << std::endl;

    for(int i = 0; i < iterations; i++){

        ne.setInputCloud (process_cloud);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (1.5);
        ne.setViewPoint(0.0,0.0,0.0);
        ne.compute (*cloud_normals);  

        est.setInputCloud (process_cloud);
        est.setInputNormals (cloud_normals);
        // float radius = 0.02 + (i * 0.02);
        float radius = 0.1;
        std::cout << "radius: " << radius << std::endl;
        est.setRadiusSearch (radius);   // 2cm radius
        est.setSearchMethod (typename pcl::search::KdTree<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
        est.compute (boundaries);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        for(int i = 0; i < process_cloud->points.size(); i++)
        {
            if(boundaries[i].boundary_point < 1)
            {
                    inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(process_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*process_cloud);

        std::cout << "proces cloud size: " << process_cloud->size() << std::endl;
    
    }

    return process_cloud;

    
}


int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    char temp[100];
    std::string ye = getcwd(temp, sizeof(temp)) ? std::string( temp ) : std::string("");

    // pcl::io::savePCDFile("./big_segment.pcd", *(clusters[i].cloud), true);
    std::cout << "current  dir: " << ye << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("big_segment.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file big_segmented.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from big_segmented.pcd with the following fields: "
        << std::endl;


    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud);

    // PointCloud<PointXYZ>::Ptr cloud_xyz;
    // copyPointCloud(*cloud_xyz, *cloud);

    // pcl::Kmeans real(static_cast<int> (cloud->points.size()), 3);
    // real.setClusterSize(3); //it is important that you set this term appropriately for your application

    // for (size_t i = 0; i < cloud->points.size(); i++)
    // {s
    //     std::vector<float> data(3);
    //     data[0] = cloud->points[i].x;
    //     data[1] = cloud->points[i].y;
    //     data[2] = cloud->points[i].z;
    //     real.addDataPoint(data);
    // }

    // real.kMeans();

    // pcl::Kmeans::Centroids centroids = real.get_centroids();
    // std::cout << "points in total Cloud : " << cloud->points.size() << std::endl;
    // std::cout << "centroid count: " << centroids.size() << std::endl;
    // for (int i = 0; i < centroids.size(); i++)
    // {
    //     std::cout << i << "_cent output: x: " << centroids[i][0] << " ,";
    //     std::cout << "y: " << centroids[i][1] << " ,";
    //     std::cout << "z: " << centroids[i][2] << std::endl;
    //     pcl::PointXYZRGB point;
    //     point.x = centroids[i][0];
    //     point.y = centroids[i][1];
    //     point.z = centroids[i][2];
    //     point.r = 0;
    //     point.g = 0;
    //     point.b = 255;
    //     cloud->points.push_back(point);
    // }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // fill in the cloud data here
    
    // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    
    // NormalEstimation<PointXYZRGB, Normal> ne;
	// ne.setInputCloud (cloud);

	// // Create an empty kdtree representation, and pass it to the normal estimation object.
	// // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	// search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
	// ne.setSearchMethod (tree);

	// // Output datasets
	// PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);

	// // Use all neighbors in a sphere of radius 3cm
	// ne.setRadiusSearch (1.5);
	// ne.setViewPoint(0.0,0.0,0.0);

	// // Compute the features
	// ne.compute (*cloud_normals);  

    // pcl::PointCloud<pcl::Boundary> boundaries;
    // pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> est;
    // est.setInputCloud (cloud);
    // est.setInputNormals (cloud_normals);
    // // float radius;
    // // cin >> radius;
    // est.setRadiusSearch (0.02);   // 2cm radius
    // est.setSearchMethod (typename pcl::search::KdTree<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
    // est.compute (boundaries);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    // for(int i = 0; i < cloud->points.size(); i++)
    // {
    //         if(boundaries[i].boundary_point < 1)
    //         {
    //                 // cloud->at(i).z = 0;
    //                 cloud2->points.push_back(cloud->at(i));
    //         }
    // }

	// PointCloud<Normal>::Ptr cloud_normals2 (new PointCloud<Normal>);

	// // Output datasets
    // ne.setInputCloud(cloud2);
	// // Use all neighbors in a sphere of radius 3cm
	// ne.setRadiusSearch (1.5);
	// ne.setViewPoint(0.0,0.0,0.0);
	// // Compute the features
	// ne.compute (*cloud_normals2);  

    // // pcl::PointCloud<pcl::Boundary> boundaries;
    // // pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> est;
    // est.setInputCloud (cloud2);
    // est.setInputNormals (cloud_normals2);
    // // float radius;
    // // cin >> radius;
    // est.setRadiusSearch (0.02);   // 2cm radius
    // est.setSearchMethod (typename pcl::search::KdTree<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
    // est.compute (boundaries);


    // // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    // for(int i = 0; i < cloud2->points.size(); i++)
    // {
    //         if(boundaries[i].boundary_point < 1)
    //         {
    //                 cloud2->at(i).z = 0;
    //                 // cloud3->points.push_back(cloud->at(i));
    //         }
    // }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = iterativeBoundaryRemoval(cloud, 4);



    viewer->updatePointCloud(cloud2, "kinectcloud");

    while(!viewer->wasStopped())
    {
        



        viewer->spinOnce();
    }
    std::cout << "hiii" << std::endl;
    return (0);
}
