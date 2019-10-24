#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>


class DepthParser
{
	private:
        cv::Point findHighestPoint(cv::Mat image);
        std::vector<cv::Point> getPointNeighbours(cv::Point p);


	public:		
		DepthParser();
        void extendDepthImageColors(cv::Mat irimage);
        cv::Mat getBestPossibleCorkPiece(cv::Mat input_image);

		~DepthParser();
		
};

