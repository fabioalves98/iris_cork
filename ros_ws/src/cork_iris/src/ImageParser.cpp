#include "ImageParser.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>

ImageParser::ImageParser(void)
{
 
}

ImageParser::~ImageParser(void)
{

}




cv::Mat ImageParser::thresholdImage(cv::Mat image, int thresholdValue)
{
    cv::Mat newimg;
    
    cv::cvtColor(image, newimg, CV_BGR2GRAY);
    cv::GaussianBlur(newimg, newimg, cv::Size(5, 5), 0, 0);
    cv::threshold(newimg, newimg, thresholdValue, 255, cv::THRESH_BINARY);
    return newimg;

}


std::vector<std::vector<cv::Point>> ImageParser::parseImageContours(cv::Mat image, int thresholdValue)
{
    cv::Mat i;
    i = ImageParser::thresholdImage(image, thresholdValue);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat hierarchy;

    cv::findContours(i, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    hierarchy.release();
    return contours;
}

std::vector<std::vector<cv::Point>> ImageParser::filterContoursByArea(std::vector<std::vector<cv::Point>> contours, int min_area, int max_area)
{
    std::vector<std::vector<cv::Point>> filtered_contours;

    for( int i = 0; i < contours.size(); i++ )
    {
        int area = cv::contourArea(contours[i]);
        if(area < max_area && area > min_area)
        {
            filtered_contours.push_back(contours[i]);
        }
    }

    return filtered_contours;
}

std::vector<cv::RotatedRect> ImageParser::getContourBoundingBox(std::vector<std::vector<cv::Point>> contours)
{
    std::vector<cv::RotatedRect> minRect(contours.size());
    for(int i = 0; i < contours.size(); i++){
        minRect[i] = cv::minAreaRect(contours[i]);
    }
    return minRect;
}


int ImageParser::getImageGrayMean(cv::Mat image){

    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    cv::Scalar m = cv::mean(gray);
    int media = (int)m[0];
    // printf("%d\n", media);
    return media;

}





