#include <stdlib.h>

#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/conversions.h>

#include <cork_iris/PCLCorkConfig.h>

using std::cout;
using std::endl;

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudNormal;
typedef CloudNormal::Ptr CloudNormalPtr;
typedef pcl::IndicesClusters IdxClusters;
typedef pcl::IndicesClustersPtr IdxClustersPtr;
typedef pcl::RGB Color;
typedef int Index;

struct CECExtractionParams{
    double normal_diff;
    double squared_dist;
    double curv;
    double leaf_size;
    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
};

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

// Parameters
static double roller_height_value;
static double roller_height_angle;
static double roller_width_value;

static int meanK;

static double radius_search;

static CECExtractionParams cec_params;

class PCLFunctions
{
private:
    static bool enforceNormals(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance);

public:
    static void updateParams(cork_iris::PCLCorkConfig &config);
    static void filterRollerBox(CloudPtr cloud_in, CloudPtr cloud_out);
    static void remove_outliers(CloudPtr cloud_in, CloudPtr cloud_out);
    static void cloud_smoothing(CloudPtr cloud_in, CloudPtr cloud_out);
    static void surface_normals(CloudPtr cloud_in, CloudPtr cloud_out);
    static void surface_curvatures(CloudPtr cloud_in, CloudPtr cloud_out);
    static void cec_extraction(CloudPtr cloud_in, CloudPtr cloud_out, IdxClustersPtr clusters, IdxClustersPtr sclusters, IdxClustersPtr lclusters, CloudNormalPtr cloud_with_normals);
    static BoundingBox computeCloudBoundingBox(CloudPtr cloud_in);
    static CloudPtr subtractCloud(CloudPtr cloud, pcl::PointIndices::Ptr indices);
    static void getNearestNeighbors(int K, Eigen::Vector4f searchPoint, CloudPtr cloud, std::vector<int>& points, std::vector<float>& dists);
    static pcl::PCLImage extractImageFromCloud(CloudPtr cloud_in, bool paintNaNBlack);
};


