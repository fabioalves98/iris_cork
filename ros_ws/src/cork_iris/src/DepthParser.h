#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>


class DepthParser
{
	private:
        std::vector<cv::Point> getPointNeighbours(cv::Point p);
        int minval, maxval;
        cv::Mat image;

	public:		
		DepthParser(cv::Mat image);

        void extendDepthImageColors(std::vector<cv::Point> contour);
        cv::Point findMinMaxPoint(std::vector<cv::Point> contour, bool toggle);
        std::vector<cv::Point> getBestPossibleCorkPiece(std::vector<cv::Point> contour);

		~DepthParser();
		
};

