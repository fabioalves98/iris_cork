#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>

class Box
{
    private:
        cv::Mat image;
    public:
        Box(cv::Mat image);
        std::vector<cv::Point> get_pins();
        void draw_rect(std::vector<cv::Point> pins);
        std::vector<cv::Point> get_blue_box();

        std::vector<cv::Point> get_box_corners(std::vector<cv::Point> box_contour); 
        cv::Mat getMaskInRange(cv::Scalar min, cv::Scalar max);
       
};