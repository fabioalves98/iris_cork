#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "ros/ros.h"
#include "ImageParsing.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

void drawImageContours(Mat drawing, std::vector<std::vector<cv::Point>> contours)
{
    for( int i = 0; i < contours.size(); i++ )
    {
        drawContours(drawing, contours, i, Scalar(0, 255, 0), 1);
    }
}

void drawContourBoundingBox(Mat drawing, std::vector<cv::RotatedRect> rects)
{
    for(int i = 0; i < rects.size(); i++){
        Point2f rect_points[4];
        rects[i].points(rect_points);
        for(int j = 0; j < 4; j++){
            line(drawing, rect_points[j], rect_points[(j+1) % 4], Scalar(0, 255, 0));
        }
    }
}


int main(int argc, char **argv){

    char* filename = argv[1];
    printf("Loading: %s\n", filename);
    FileStorage fs(filename, FileStorage::READ);
    ImageParsing ip;
    Mat loadedimg;
    fs["img"] >> loadedimg;

    // ip.extendDepthImageColors(loadedimg);
    // loadedimg = ip.thresholdImage(loadedimg, 50);
    std::vector<std::vector<cv::Point>> conts;
    conts = ip.parseImageContours(loadedimg, 50);
    conts = ip.filterContoursByArea(conts, 500, 10000);



    // draw those contours
    drawImageContours(loadedimg, conts);
    std::vector<cv::RotatedRect> rect_points;
    rect_points = ip.getContourBoundingBox(conts);
    drawContourBoundingBox(loadedimg, rect_points);

    while(1){
        imshow("Display", loadedimg);

        // wait for quit key
        if(waitKey(15) == 27 && 0xFF) break;

    }

}
