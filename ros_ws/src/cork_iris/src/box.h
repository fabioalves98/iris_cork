#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>

class Box
{
    public:
        Box();
        std::vector<cv::Point> get_pins(cv::Mat image);
        void draw_rect(cv::Mat image, std::vector<cv::Point> pins);
        std::vector<cv::Point> get_blue_box(cv::Mat image);

        std::vector<cv::Point> get_box_corners(std::vector<cv::Point> box_contour); 
        cv::Mat getMaskInRange(cv::Mat image, cv::Scalar min, cv::Scalar max);
       
};