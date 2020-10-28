#include <stdlib.h>

#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudNormal;
typedef CloudNormal::Ptr CloudNormalPtr;
typedef pcl::RGB Color;

class PCLFunctions
{
private:

public:
    static void filterRollerBox(CloudPtr cloud_in, CloudPtr cloud_out, float width_value, float heigth_value, float angle_value);
    static void remove_outliers(CloudPtr cloud_in, CloudPtr cloud_out, int meanK);
    static void cloud_smoothing(CloudPtr cloud_in, CloudPtr cloud_out);
    static void surface_normals(CloudPtr cloud_in, CloudPtr cloud_out, double radius);
    static void surface_curvatures(CloudPtr cloud_in, CloudPtr cloud_out, double radius);
};


