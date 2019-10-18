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


std::vector<cv::Point> spiral_out(int x_, int y_, int xOffset, int yOffset){
    std::vector<cv::Point> spiralPoints;
    int x,y,dx,dy;
    x = y = dx = 0;

    dy = -1;
    int t = std::max(x_,y_);
    int maxI = t*t;
    for(int i = 0; i < maxI; i++){
        if ((-x_/2 <= x) && (x <= x_/2) && (-y_/2 <= y) && (y <= y_/2)){        
            // printf("%d, %d\n", x+xOffset, y+yOffset);
            spiralPoints.push_back(Point(x+xOffset, y+yOffset));
        }
        if( (x == y) || ((x < 0) && (x == -y)) || ((x > 0) && (x == 1-y))){
            t = dx;
            dx = -dy;
            dy = t;
        }
        x += dx;
        y += dy;
    }

    return spiralPoints;
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

    // Draw a rectangle arround the boxs pins
    // Box box;
    // std::vector<Point> good_pins = box.get_pins(loadedimg);
    // // cout << good_pins << endl;
    // box.draw_rect(loadedimg, good_pins);


    

    // int media = ip.getImageGrayMean(loadedimg);

    ip.extendDepthImageColors(loadedimg);
    // loadedimg = ip.thresholdImage(loadedimg, 50);


    int min = 256;
    int x, y;
    int step = loadedimg.step;
    printf("Rows: %d  Cols: %d  Step: %d\n", loadedimg.rows, loadedimg.cols, step);
    // Hard coded starting j and i values just for testing
    unsigned char *input = (unsigned char*)(loadedimg.data);
    for(int j = 200;j < loadedimg.rows-100;j++){
        for(int i = 150;i < loadedimg.cols-50;i++){
            unsigned char b = input[loadedimg.step * j + i ] ;
            unsigned char g = input[loadedimg.step * j + i + 1];
            unsigned char r = input[loadedimg.step * j + i + 2];

            // printf("(%d, %d, %d)\n", r, g, b);
            // why do some pixels have different values rgb values? (0, 255, 0) ... etc
            if(r == g && r == b){
                if(r < min && r != 0){
                    min = r;
                    x = i;
                    y = j;
                }
            }

        }
    }
    // cvtColor(loadedimg, loadedimg, CV_GRAY2BGR);
    // printf("New pixel found! (%d, %d)[%d]\n", x, y, min);
    // circle(loadedimg, Point(x, y), 4, Scalar(0, 0, 255));
    vector<Point> spiralPoints = spiral_out(100, 100, x, y);

    for(int i = 0; i < spiralPoints.size(); i++){
        Point p = spiralPoints.at(i);
        unsigned char r = input[loadedimg.step * p.y + p.x + 2];
        cout << p;
        printf("%d\n", r);
        // circle(loadedimg, p, 1, Scalar(255, 0, 0));
    }

    



    // std::vector<std::vector<cv::Point>> conts;
    // int i = media;
    // cv::putText(loadedimg, std::to_string(i), cv::Point(20, 50),
    //             cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 0));
    while(1){


        // Get contours and filter them by a certain area
        // conts = ip.parseImageContours(loadedimg, i);
        // conts = ip.filterContoursByArea(conts, 500, 8000);

        // Draw contours and their bounding boxes
        // drawImageContours(loadedimg, conts);
        // std::vector<cv::RotatedRect> rect_points;
        // rect_points = ip.getContourBoundingBox(conts);
        // drawContourBoundingBox(loadedimg, rect_points);

        imshow("Display", loadedimg);

        // ESC
        if(waitKey(0) == 27 && 0xFF) break;
        // C Key
        // else if(waitKey(0) == 99 && 0xFF) {
        //     if(i < 250) i+=5;
        // // x_ Key
        // }else if(waitKey(0) == 120 && 0xFF){
        //     if(i > 30) i-=5;
        // }

        // Clear all contours, reload the image and update text
        // conts.clear();
        // fs["img"] >> loadedimg;
        // cv::putText(loadedimg, std::to_string(i), cv::Point(20, 50),
        //             cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 0));

    }

}
