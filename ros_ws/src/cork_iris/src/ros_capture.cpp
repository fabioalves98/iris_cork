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


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>

using namespace cv;

pthread_mutex_t mutex_kinect = PTHREAD_MUTEX_INITIALIZER;
pthread_t cv_thread;

cv::Mat global_img;
int first_assignment = 0;


void image_callback(const sensor_msgs::ImageConstPtr& msg){
    pthread_mutex_lock( &mutex_kinect );
        // printf("callback\n");

    cv_bridge::CvImagePtr cv_image_ptr;

    try{
        cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        global_img = cv_image_ptr->image;
        first_assignment = 1;
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("%s", e.what());
        return;
    }
    // imshow("Display", cv_image_ptr->image);
    // waitKey(3);
    pthread_mutex_unlock( &mutex_kinect );


}

void *cv_threadfunc (void *ptr) {

    // use image polling
    while (1)
    {
        // printf("display thread");
        //lock mutex for depth image
        pthread_mutex_lock( &mutex_kinect );
        // cvtColor(depthimg,tempimg,CV_GRAY2BGR);
        if(first_assignment){
            namedWindow("Display");
            cv::Mat img_scaled_8u;
            cv::Mat(global_img-0).convertTo(img_scaled_8u, CV_8UC1, 255. / (1000 - 0));
            // cv::cvtColor(img_scaled_8u, global_img, CV_GRAY2RGB);
            imshow("Display", global_img);
        }
        //unlock mutex for depth image
        pthread_mutex_unlock( &mutex_kinect );


        // wait for quit key
        if(waitKey(15) == 27 && 0xFF){
            destroyWindow("Display");
            break;
        }
        else if(waitKey(15) == 99 & 0xFF){
            FileStorage file("img.ext", cv::FileStorage::WRITE);
            file << "img" << global_img;
        }

    }
    pthread_exit(NULL);

    return NULL;

}

int main(int argc, char** argv){

    ros::init(argc, argv, "ros_capture");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    // image_transport::Publisher image_pub;

    image_sub = it.subscribe("/camera/rgb/image_raw", 1, image_callback);
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