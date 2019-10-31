#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>


class DepthParser
{
	private:
        std::vector<cv::Point> getPointNeighbours(cv::Point p);

	public:		
		DepthParser();
        void extendDepthImageColors(cv::Mat irimage, std::vector<cv::Point> contour);
        cv::Point findMinMaxPoint(cv::Mat image, std::vector<cv::Point> contour, bool toggle);
        cv::Mat getBestPossibleCorkPiece(cv::Mat input_image, std::vector<cv::Point> contour);

		~DepthParser();
		
};

