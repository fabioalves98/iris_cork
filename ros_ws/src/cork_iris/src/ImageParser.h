#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>


class ImageParser
{
	private:
        cv::Mat image;

	public:		
		ImageParser(cv::Mat image);
        cv::Mat thresholdImage(int thresholdValue);
        std::vector<std::vector<cv::Point>> parseImageContours(int thresholdValue);
        std::vector<std::vector<cv::Point>> filterContoursByArea(std::vector<std::vector<cv::Point>> contours, int min_area, int max_area);
        std::vector<cv::RotatedRect> getContourBoundingBox(std::vector<std::vector<cv::Point>> contours);
        std::vector<cv::Point> smallestAreaContour(std::vector<std::vector<cv::Point>> contours);



        int getImageGrayMean(cv::Mat image);

		~ImageParser();
		
};

