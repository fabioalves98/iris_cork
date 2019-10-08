#include <stdio.h>
#include <string.h>
#include <math.h>
#include <libfreenect.h>
#include <pthread.h>
#include "ros/ros.h"
// #define CV_NO_BACKWARD_COMPATIBILITY
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;


int main(int argc, char **argv){

    FileStorage fs("rgb.ext", FileStorage::READ);

    Mat loadedimg;
    fs["rgb"] >> loadedimg;

    while(1){
        imshow("Display", loadedimg);

        // wait for quit key
        if(waitKey(15) == 27 && 0xFF) break;

    }

}
