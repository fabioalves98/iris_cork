#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "ros/ros.h"
#include "ImageParser.h"
#include "box.h"
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
    if(argc != 2){
        printf("No file to load provided!\n");
        return 0;
    }

    printf("Loading: %s\n", filename);
    FileStorage fs(filename, FileStorage::READ);
    ImageParser ip;
    Mat loadedimg;
    fs["img"] >> loadedimg;

    Box box;
    std::vector<Point> good_pins = box.get_pins(loadedimg);

    cout << good_pins << endl;

    box.draw_rect(loadedimg, good_pins);

    int media = ip.getImageGrayMean(loadedimg);

    // ip.extendDepthImageColors(loadedimg);
    // loadedimg = ip.thresholdImage(loadedimg, 50);

    // Rect roi(95, 70 , 400, 380);
    // loadedimg = loadedimg(roi);

    std::vector<std::vector<cv::Point>> conts;
    int i = media;
    cv::putText(loadedimg, std::to_string(i), cv::Point(20, 50),
                cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 0));
    while(1){


        // Get contours and filter them by a certain area
        conts = ip.parseImageContours(loadedimg, i);
        conts = ip.filterContoursByArea(conts, 500, 8000);

        // Draw contours and their bounding boxes
        drawImageContours(loadedimg, conts);
        std::vector<cv::RotatedRect> rect_points;
        rect_points = ip.getContourBoundingBox(conts);
        drawContourBoundingBox(loadedimg, rect_points);

        imshow("Display", loadedimg);

        // ESC
        if(waitKey(0) == 27 && 0xFF) break;
        // C Key
        else if(waitKey(0) == 99 && 0xFF) {
            if(i < 250) i+=5;
        // X Key
        }else if(waitKey(0) == 120 && 0xFF){
            if(i > 30) i-=5;
        }

        // Clear all contours, reload the image and update text
        conts.clear();
        fs["img"] >> loadedimg;
        cv::putText(loadedimg, std::to_string(i), cv::Point(20, 50),
                    cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 0));

    }

}
