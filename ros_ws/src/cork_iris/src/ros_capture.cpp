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
pthread_t cv_thread;

cv::Mat global_img_depth, global_img_rgb;
cv::Mat img_scaled_8u;

bool depth_assignment = 0;
bool rgb_assignment = 0;
bool parse_image = 0;

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



void depth_callback(const sensor_msgs::ImageConstPtr& msg){
    pthread_mutex_lock( &mutex_kinect );

    cv_bridge::CvImagePtr cv_image_ptr;

    try{
        cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        // cv_image_ptr = cv_bridge::toCvCopy(msg);
        global_img_depth = cv_image_ptr->image;
        depth_assignment = 1;
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("%s", e.what());
        return;
    }

    pthread_mutex_unlock( &mutex_kinect );


}

void rgb_callback(const sensor_msgs::ImageConstPtr& msg){
    pthread_mutex_lock( &mutex_kinect );

    cv_bridge::CvImagePtr cv_image_ptr;

    try{
        cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        global_img_rgb = cv_image_ptr->image;
        rgb_assignment = 1;
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("%s", e.what());
        return;
    }
    pthread_mutex_unlock( &mutex_kinect );


}

void *cv_threadfunc (void *ptr) {

    // use image polling
    while (1)
    {
        //lock mutex for depth image
        pthread_mutex_lock( &mutex_kinect );
        if(depth_assignment){
            namedWindow("Depth Display");
            cv::Mat(global_img_depth-0).convertTo(img_scaled_8u, CV_8UC1, 255. / (1000 - 0));
            // cv::cvtColor(img_scaled_8u, global_image_depth, CV_GRAY2RGB);
            // std::cout << img_scaled_8u.rows << img_scaled_8u.step << img_scaled_8u.cols << std::endl;
            // Draw a rectangle arround the box's pins
            //std::vector<Point> good_pins = box.get_pins(global_image_depth);
            //box.draw_rect(global_image_depth, good_pins);



            // 
            // cv::Point hp = dp.findHighestPoint(img_scaled_8u);
            // std::cout << hp << std::endl;
            // circle(global_img_rgb, hp, 4, Scalar(0, 0, 255));
            // circle(img_scaled_8u, hp, 4, Scalar(0, 0, 255));
            
           
            imshow("Depth Display", img_scaled_8u);
        }

        if(rgb_assignment){
            if(parse_image){
                cv::Mat parsed_image = global_img_rgb.clone();
                //cv::Mat(global_img_rgb-0).convertTo(img_scaled_8u, CV_8UC1, 255. / (1000 - 0));
                cv::Mat drawable = global_img_rgb.clone();
                std::vector<Point> points = box.get_blue_box(drawable);
                cv::Mat mask = box.getMaskInRange(drawable, cv::Scalar(0, 0, 250), cv::Scalar(0, 0, 255));

                int min_area = 1000;
                int max_area = 200000;
                std::vector<std::vector<Point>> contour_points;
                // This contour should be the inside contour (excluding the all the box around the cork pieces)
                contour_points.push_back(ip.smallestAreaContour(ip.filterContoursByArea(ip.parseImageContours(mask, -1), min_area, max_area)));
                drawImageContours(parsed_image, contour_points);
                std::vector<cv::Point> corners = box.get_box_corners(contour_points.at(0));
                bool isInside = cv::pointPolygonTest(contour_points.at(0), Point(320, 240), false);
                //imshow("mask Display", mask);
                imshow("parsed Display", parsed_image);
                parse_image = 0;
            }
            namedWindow("RGB Display");
            
            // for (int i = 0; i < corners.size(); i++)
            // {
            //     circle(global_img_rgb, corners.at(i), 3, cv::Scalar(0, 255, 255), 2);
            // }
            
            // Not live
    
            // Live feed
            imshow("RGB Display", global_img_rgb);

        }

        //unlock mutex for depth image
        pthread_mutex_unlock( &mutex_kinect );


        // wait for quit key
        if(waitKey(15) == 27 && 0xFF){
            destroyWindow("Depth Display");
            destroyWindow("RGB Display");
            break;
        }
        else if(waitKey(15) == 99 & 0xFF){
            FileStorage file("img.ext", cv::FileStorage::WRITE);
            file << "img" << global_img_rgb;
        }else if(waitKey(15) == 100 & 0xFF){
            FileStorage file("depth.ext", cv::FileStorage::WRITE);
            file << "img" << img_scaled_8u;   
        }else if(waitKey(15) == 112 & 0xFF){
            printf("Parsing current frame...\n");
            parse_image = 1;
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
    int res = 0;

    res = pthread_create(&cv_thread, NULL, cv_threadfunc, NULL);
    if (res)
    {
        printf("pthread_create failed\n");
        return 1;
    }
    printf("Started open cv thread and all inits.\n");

    ros::spin();
    destroyWindow("Display");
    printf("window destroyed\n");
    return 0;

}