#include <stdlib.h>

#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/voxel_grid.h>

#include <cork_iris/PCLCorkConfig.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudNormal;
typedef CloudNormal::Ptr CloudNormalPtr;
typedef pcl::IndicesClustersPtr IndxClustersPtr;
typedef pcl::RGB Color;

struct CECExtractionParams{
    double normal_diff;
    double squared_dist;
    double curv;
    double leaf_size;
    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
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
    static void cec_extraction(CloudPtr cloud_in, CloudPtr cloud_out, IndxClustersPtr clusters, IndxClustersPtr sclusters, IndxClustersPtr lclusters, CloudNormalPtr cloud_with_normals);
};


