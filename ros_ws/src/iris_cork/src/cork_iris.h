#include <stdlib.h>

#include "pcl_functions.h"

using std::cout;
using std::endl;
using std::uint8_t;

static std::vector<std::string> colors = {"orange", "red", "green", "yellow", "blue", "pink", "cyan"};


// From https://colordesigner.io/gradient-generator
// rgb(0, 255, 126)
// rgb(117, 218, 55)
// rgb(149, 181, 0)
// rgb(164, 142, 0)
// rgb(167, 103, 0)
// rgb(161, 61, 0)
// rgb(146, 0, 0)
static std::vector<std::vector<int>> gradient_colors = {{0, 255, 255}, {76, 255, 147}, {150, 191, 20},
{255, 160, 0}, {255, 0, 0}};

static bool choose_best_cork;
static Index chosen_cork_strip;
static double z_threshold, center_threshold;
static double space_distance_threshold, space_count_points_threshold, space_k_neighbors;
static double bad_shape_width_threshold, volume_threshold;
static double splitted_cork_distance_threshold, splitted_cork_normal_threshold;
static double height_weight, distance_weight, size_weight;

struct CloudInfo{
    CloudPtr cloud;
    CloudNormalPtr cloudNormal;
    pcl::PointIndices::Ptr indices;
    BoundingBox bb;
    
    // Choose best cluster related parameters 
    float average_height;
    float cloud_size;
    float distance_manipulator;

    float height_score;
    float size_score;
    float distance_score;

    float score;
};

class CorkIris
{
private:
    static std::vector<CloudInfo> clusterIndicesToCloud(IdxClustersPtr clusters, CloudPtr original_cloud, CloudNormalPtr original_cloud_normal);
    static std::vector<CloudInfo> segmentBigClusters(CloudPtr cloud_in, CloudNormalPtr cloud_with_normals, std::vector<CloudInfo> clusters);
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
    static Index getBiggestCluster(std::vector<CloudInfo> clusters);
    static Index getBestClusterToGrab(std::vector<CloudInfo> clusters);
    static bool isThereSpace(CloudInfo cluster, CloudPtr fullCloud);

public:
    static void updateParams(iris_cork::PCLCorkConfig &config);
    static CloudInfo clusterExtraction(CloudPtr cloud_in, CloudPtr cloud_out);
};
