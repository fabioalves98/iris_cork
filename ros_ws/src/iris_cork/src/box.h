#include <stdlib.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/io.h>

class Box
{
    private:
        static std::vector<cv::Point> getCorkContours(cv::Mat cv_image);
        static cv::Mat get_blue_box(cv::Mat cv_image);
        static cv::Mat getMaskInRange(cv::Mat cv_image, cv::Scalar min, cv::Scalar max);
    public:
        static void removeBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, 
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
                       cv::Mat cv_image);
};