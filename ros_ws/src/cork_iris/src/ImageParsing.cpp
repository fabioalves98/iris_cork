#include "ImageParsing.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>

ImageParsing::ImageParsing(void)
{
 
}

ImageParsing::~ImageParsing(void)
{

}


void ImageParsing::extendDepthImageColors(cv::Mat image)
{
    int minval = 154;
    int maxval = 185;
    int diff = maxval-minval;
    int i, j;
    unsigned char *ptr = (unsigned char*)(image.data);
    for(i = 0; i < image.cols; i++){
        for(j = 0; j < image.rows; j++){
            if(ptr[image.cols * j + i] < minval){
                ptr[image.cols * j + i] = 0;
            }else if(ptr[image.cols * j + i] >= maxval){
                ptr[image.cols * j + i] = 255;           
            }else{
                ptr[image.cols * j + i] = (ptr[image.cols * j + i] - minval) * (255 / diff);
   
            }
        }
    }
    
}

cv::Mat ImageParsing::thresholdImage(cv::Mat image, int thresholdValue)
{
    cv::Mat newimg;
    
    cv::cvtColor(image, newimg, CV_BGR2GRAY);
    cv::GaussianBlur(newimg, newimg, cv::Size(5, 5), 0, 0);
    cv::threshold(newimg, newimg, thresholdValue, 255, cv::THRESH_BINARY);
    return newimg;

}


std::vector<std::vector<cv::Point>> ImageParsing::parseImageContours(cv::Mat image, int thresholdValue)
{
    cv::Mat i;
    i = ImageParsing::thresholdImage(image, thresholdValue);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat hierarchy;

    cv::findContours(i, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    hierarchy.release();
    return contours;
}

std::vector<std::vector<cv::Point>> ImageParsing::filterContoursByArea(std::vector<std::vector<cv::Point>> contours, int min_area, int max_area)
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

std::vector<cv::RotatedRect> ImageParsing::getContourBoundingBox(std::vector<std::vector<cv::Point>> contours)
{
    std::vector<cv::RotatedRect> minRect(contours.size());
    for(int i = 0; i < contours.size(); i++){
        minRect[i] = cv::minAreaRect(contours[i]);
    }
    return minRect;
}





