#include <stdlib.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/io.h>

#include "ImageParser.h"

class Box
{
    private:
        cv::Mat image;
        static std::vector<cv::Point> getCorkContours(cv::Mat cv_image);

    public:
        Box(cv::Mat image);
        static void removeBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, 
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
                       cv::Mat cv_image);

        std::vector<cv::Point> get_blue_box();

        std::vector<cv::Point> get_box_corners(std::vector<cv::Point> box_contour); 
        cv::Mat getMaskInRange(cv::Scalar min, cv::Scalar max);
       
};