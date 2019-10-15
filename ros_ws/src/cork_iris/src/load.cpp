#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "ros/ros.h"
#include "ImageParser.h"
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
    if(argc < 2){
        printf("No file to load provided!\n");
        return 0;
    }
    printf("Loading: %s\n", filename);
    FileStorage fs(filename, FileStorage::READ);
    ImageParser ip;
    Mat loadedimg;
    fs["img"] >> loadedimg;
    Mat gray;
    cvtColor(loadedimg, gray, CV_BGR2GRAY);
    Scalar m = mean(gray);
    int media = (int)m[0];
    printf("%d\n", media);

    // ip.extendDepthImageColors(loadedimg);
    // loadedimg = ip.thresholdImage(loadedimg, 50);
    std::vector<std::vector<cv::Point>> conts;
    // conts = ip.parseImageContours(loadedimg, 50);
    // conts = ip.filterContoursByArea(conts, 500, 10000);
    // Rect roi(95, 70 , 400, 380);
    // loadedimg = loadedimg(roi);

    // draw those contours
    // drawImageContours(loadedimg, conts);
    // std::vector<cv::RotatedRect> rect_points;
    // rect_points = ip.getContourBoundingBox(conts);
    // drawContourBoundingBox(loadedimg, rect_points);
    
    int i = 50;
    while(1){

        conts = ip.parseImageContours(loadedimg, media);
        conts = ip.filterContoursByArea(conts, 500, 8000);

        // draw those contours
        drawImageContours(loadedimg, conts);
        std::vector<cv::RotatedRect> rect_points;
        rect_points = ip.getContourBoundingBox(conts);
        drawContourBoundingBox(loadedimg, rect_points);
        imshow("Display", loadedimg);

        // wait for quit key
        if(waitKey(0) == 27 && 0xFF) break;
        else if(waitKey(0) == 99 && 0xFF) {

         
            conts.clear();
            fs["img"] >> loadedimg;
            // loadedimg = loadedimg(roi);
               cv::putText(loadedimg, //target image
            std::to_string(i), //text
            cv::Point(10, loadedimg.rows / 2), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            CV_RGB(118, 185, 0), //font color
            2);
            if(i < 250) i+=5;
            else break;
        } 

    }

}
