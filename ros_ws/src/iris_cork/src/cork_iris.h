#include <stdlib.h>

#include "pcl_functions.h"

using std::cout;
using std::endl;

static std::vector<std::string> colors = {"orange", "red", "green", "yellow", "blue", "pink", "cyan"};

static bool choose_best_cork;
static Index chosen_cork_strip;
static double z_threshold, center_threshold;
static double space_distance_threshold, space_count_points_threshold, space_k_neighbors;
static double bad_shape_percentage_threshold, volume_threshold;
static double splitted_cork_distance_threshold, splitted_cork_normal_threshold;

class CorkIris
{
private:
    static std::vector<CloudInfo> clusterIndicesToCloud(IdxClustersPtr clusters, CloudPtr original_cloud, CloudNormalPtr original_cloud_normal);
    static std::vector<CloudInfo> segmentBigClusters(std::vector<CloudInfo> clusters);
    static bool isClusterBadShaped(BoundingBox cluster);
    static bool isClusterTooBig(BoundingBox cluster);
    static std::vector<CloudInfo> joinSplittedClusters(std::vector<CloudInfo> clusters);
    static bool isSplittedCluster(CloudInfo cluster0, CloudInfo cluster1);
    static Eigen::Vector3f getClusterAverageNormal(CloudNormalPtr cluster);
    static CloudInfo joinClusters(CloudInfo cluster0, CloudInfo cluster1);
    static void paintClustersFull(CloudPtr cloud_out, std::vector<CloudInfo> clusters, IdxClustersPtr small_clusters, IdxClustersPtr large_clusters);
    static void paintClusters(CloudPtr cloud_out, std::vector<CloudInfo> clusters);
    static CloudInfo chooseBestCluster(std::vector<CloudInfo> cluster_clouds, CloudPtr fullPointCloud);
    static Index getHighestCluster(std::vector<CloudInfo> clusters);
    static bool isThereSpace(CloudInfo cluster, CloudPtr fullCloud);

public:
    static void updateParams(iris_cork::PCLCorkConfig &config);
    static CloudInfo clusterExtraction(CloudPtr cloud_in, CloudPtr cloud_out);
};
