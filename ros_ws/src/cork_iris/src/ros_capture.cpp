/*
 FROM: http://wiki.ros.org/image_transport
 image_transport Publishers
    4.1:
    Image_transport publishers are used much like ROS Publishers,
    but may offer a variety of specialized transport options 
    (JPEG compression, streaming video, etc.). 
    Different subscribers may request images from the same 
    publisher using different transports. 

*/

#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>
#include "box.h"
#include "DepthParser.h"
#include "ImageParser.h"

using namespace cv;

pthread_mutex_t mutex_kinect = PTHREAD_MUTEX_INITIALIZER;
pthread_t cv_thread, key_press, image_processing;


cv::Mat raw_global_img_depth, global_img_depth, global_img_rgb;
cv::Mat drawable;

bool depth_assignment = 0;
bool rgb_assignment = 0;
bool drawable_assignment = 0;

Box box;
DepthParser dp;
ImageParser ip;

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

void *process_image(void* args){    
    cv::Mat parsed_image = global_img_rgb.clone();
    cv::Mat parsed_depth = global_img_depth.clone();
    
    // Image box
    cv::Mat box_image =  global_img_rgb.clone();
    std::vector<Point> points = box.get_blue_box(box_image);
    cv::Mat mask = box.getMaskInRange(box_image, cv::Scalar(0, 0, 250), cv::Scalar(0, 0, 255));

    
    // Contours
    int min_area = 20000;
    int max_area = 200000;
    std::vector<std::vector<Point>> contour_points;

    // This contour should be the inside contour (excluding the all the box around the cork pieces)
    contour_points.push_back(ip.smallestAreaContour(ip.filterContoursByArea(ip.parseImageContours(mask, -1), min_area, max_area)));
    drawImageContours(parsed_image, contour_points);
    
    // dp.extendDepthImageColors(parsed_depth, contour_points.at(0));
    cv::Mat cork_piece = dp.getBestPossibleCorkPiece(parsed_depth, contour_points.at(0));
    
    
    drawable = cork_piece.clone();    
    drawable_assignment = 1;
    
    pthread_exit(NULL);

    return NULL;
}



void depth_callback(const sensor_msgs::ImageConstPtr& msg){

    cv_bridge::CvImagePtr cv_image_ptr;

    try{
        cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        raw_global_img_depth = cv_image_ptr->image;
        depth_assignment = 1;
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("%s", e.what());
        return;
    }
 
}

void rgb_callback(const sensor_msgs::ImageConstPtr& msg){

    cv_bridge::CvImagePtr cv_image_ptr;

    try{
        cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        global_img_rgb = cv_image_ptr->image;
        rgb_assignment = 1;
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("%s", e.what());
        return;
    }


}

void *cv_threadfunc (void *ptr) {
    // use image polling
    while (1)
    {
        
        if(depth_assignment)
        {
            cv::Mat(raw_global_img_depth-0).convertTo(global_img_depth, CV_8UC1, 255. / (1000 - 0));
            imshow("Depth Display", global_img_depth);
        }

        if(rgb_assignment)
        {
            imshow("RGB Display", global_img_rgb);
        }

        if(drawable_assignment)
        {
            imshow("Drawable", drawable);
            drawable_assignment = 0;
        }


        // Esc to quit
        if(waitKey(15) == 27 && 0xFF){
            destroyAllWindows();
            break;
        }

    }
    pthread_exit(NULL);

    return NULL;

}

void *key_pressfunc (void *ptr) {
    while(1)
    {
        char key;
        std::cout << "Key: ";
        std::cin >> key;
        if(key == 'p'){
            int thread_res = 0;
            thread_res = pthread_create(&image_processing, NULL, process_image, NULL);
            if(thread_res) return NULL;
        }
    }
    pthread_exit(NULL);
    return NULL;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "ros_capture");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber depth_sub;
    image_transport::Subscriber image_sub;
    // image_transport::Publisher image_pub;

    // image_sub = it.subscribe("/camera/rgb/image_raw", 1, image_callback2);
    image_sub = it.subscribe("/camera/rgb/image_raw", 1, rgb_callback);
    depth_sub = it.subscribe("/camera/depth_registered/image_raw", 1, depth_callback);
    
    // Live Image Thread
    int res_cv = 0;
    res_cv = pthread_create(&cv_thread, NULL, cv_threadfunc, NULL);
    if (res_cv)
    {
        printf("pthread_create failed\n");
        return 1;
    }
    
    int res_key = 0;
    res_key = pthread_create(&key_press, NULL, key_pressfunc, NULL);
    if(res_key)
    {
        return 1;
    }

    
    printf("Started live feed thread\n");

    ros::spin();
    destroyWindow("Display");
    printf("window destroyed\n");
    return 0;

}