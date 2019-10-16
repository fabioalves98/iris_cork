#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>


class ImageParser
{
	private:


	public:		
		ImageParser();
        void extendDepthImageColors(cv::Mat irimage);
        cv::Mat thresholdImage(cv::Mat image, int thresholdValue);
        std::vector<std::vector<cv::Point>> parseImageContours(cv::Mat image, int thresholdValue);
        std::vector<std::vector<cv::Point>> filterContoursByArea(std::vector<std::vector<cv::Point>> contours, int min_area, int max_area);
        std::vector<cv::RotatedRect> getContourBoundingBox(std::vector<std::vector<cv::Point>> contours);

        int getImageGrayMean(cv::Mat image);

		~ImageParser();
		
};

