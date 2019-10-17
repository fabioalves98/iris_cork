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
        
};