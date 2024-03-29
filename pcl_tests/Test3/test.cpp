#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

typedef pcl::PointXYZRGB PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

// bool
// enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
// {
//   if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
//     return (true);
//   else
//     return (false);
// }

// bool
// enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
// {
//   Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
//   if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
//     return (true);
//   if (std::abs (point_a_normal.dot (point_b_normal)) < 0.05)
//     return (true);
//   return (false);
// }

bool
customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
  std::cout << "squared dist: " << squared_distance << std::endl;
  std::cout << "dot: " << std::abs (point_a_normal.dot (point_b_normal)) << std::endl;
  std::cout << "---- " << std::endl;

  if (squared_distance < 0.001 )
  {
    // if (std::abs (point_a.intensity - point_b.intensity) < 8.0f)
      // return (true);
    if (std::abs (point_a_normal.dot (point_b_normal)) > 0.5)
      return (true);
  }
  // else
  // {
  //   if (std::abs (point_a.intensity - point_b.intensity) < 3.0f)
  //     return (true);
  // }
  return (false);
}

int
main (int argc, char** argv)
{
  // Data containers used
  pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
  pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);
  pcl::console::TicToc tt;

  // Load the input point cloud
  std::cerr << "Loading...\n", tt.tic ();
  pcl::io::loadPCDFile ("big_segment.pcd", *cloud_in);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->size () << " points\n";

  // Downsample the cloud using a Voxel Grid class
  std::cerr << "Downsampling...\n", tt.tic ();
  pcl::VoxelGrid<PointTypeIO> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.01, 0.01, 0.01);
  vg.setDownsampleAllData (true);
  vg.filter (*cloud_out);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->size () << " points\n";

  // Set up a Normal Estimation class and merge data in cloud_with_normals
  std::cerr << "Computing normals...\n", tt.tic ();
  pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
  pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
  ne.setInputCloud (cloud_out);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (3.0);
  ne.compute (*cloud_with_normals);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  // Set up a Conditional Euclidean Clustering class
  std::cerr << "Segmenting to clusters...\n", tt.tic ();
  pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
  cec.setInputCloud (cloud_with_normals);
  cec.setConditionFunction (&customRegionGrowing);
  cec.setClusterTolerance (10.0);
  cec.setMinClusterSize (cloud_with_normals->size () / 1000);
  cec.setMaxClusterSize (cloud_with_normals->size () / 5);
  cec.segment (*clusters);
  cec.getRemovedClusters (small_clusters, large_clusters);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  // Using the intensity channel for lazy visualization of the output
  // for (int i = 0; i < small_clusters->size (); ++i)
  //   for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
  //     (*cloud_out)[(*small_clusters)[i].indices[j]].intensity = -2.0;
  // for (int i = 0; i < large_clusters->size (); ++i)
  //   for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
  //     (*cloud_out)[(*large_clusters)[i].indices[j]].intensity = +10.0;
  std::cout << "size min: " << small_clusters->size() << std::endl;
  std::cout << "size max: " << large_clusters->size() << std::endl;
  std::cout << "size: " << clusters->size() << std::endl;

  for (int i = 0; i < clusters->size (); ++i)
  {
    int r = rand () % 255;
    int g = rand() % 255;
    int b = rand() % 255;
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j){
      (*cloud_out)[(*clusters)[i].indices[j]].r = r;
      (*cloud_out)[(*clusters)[i].indices[j]].g = g;
      (*cloud_out)[(*clusters)[i].indices[j]].b = b;
      }
  }

  // Save the output point cloud
  // std::cerr << "Saving...\n", tt.tic ();
  // pcl::io::savePCDFile ("output.pcd", *cloud_out);
  // std::cerr << ">> Done: " << tt.toc () << " ms\n";
   pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (cloud_out, "scene_cloud");

 while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  return (0);
}