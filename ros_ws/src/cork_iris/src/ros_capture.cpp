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
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "box.h"
#include "DepthParser.h"
#include "ImageParser.h"


using namespace cv;


image_transport::Publisher depth_pub;
image_transport::Publisher parsed_pub;



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

cv::Mat process_image(cv::Mat image, cv::Mat depth){    
	cv::Mat drawable;

    cv::Mat parsed_image = image.clone();
    cv::Mat parsed_depth = depth.clone();
    
    // Image box
    cv::Mat box_image =  image.clone();
    Box box(box_image);
    std::vector<Point> points = box.get_blue_box();
    // Get blue box painted all blue points bright red. Get the mask for all bright red points
    cv::Mat mask = box.getMaskInRange(cv::Scalar(0, 0, 250), cv::Scalar(0, 0, 255));

    
    // Contours
    int min_area = 20000;
    int max_area = 200000;
    std::vector<std::vector<Point>> contour_points;
    ImageParser ip(mask);

    // This contour should be the inside contour (excluding the all the box around the cork pieces)
    // This is a heavy assumption since we are considering that only two contours exist after the first
    // area filter, the outer box cntour and the inner box contour.
    contour_points.push_back(ip.smallestAreaContour(ip.filterContoursByArea(ip.parseImageContours(-1), min_area, max_area)));
    drawImageContours(parsed_image, contour_points);
    
    DepthParser dp(parsed_depth);
    dp.extendDepthImageColors(contour_points.at(0));
    std::vector<cv::Point> cork_piece = dp.getBestPossibleCorkPiece(contour_points.at(0));
    for(int i = 0; i < cork_piece.size(); i++){
        circle(parsed_image,  Point(cork_piece.at(i).x + 2, cork_piece.at(i).y), 0, cv::Scalar(255, 255, 255), 1);
    }
    
    drawable = parsed_image.clone();    
    
    return drawable;
}

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth)
{
	cv::Mat raw_global_img_depth, global_img_depth, global_img_rgb;
	cv::Mat drawable;

    cv_bridge::CvImagePtr cv_depth_ptr;
	cv_bridge::CvImagePtr cv_image_ptr;

    try{
		cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    	cv_image_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
		raw_global_img_depth = cv_depth_ptr->image;
		global_img_rgb = cv_image_ptr->image;

		// Convert depth enconding
		cv::Mat(raw_global_img_depth-0).convertTo(global_img_depth, CV_8UC1, 255. / (1000 - 0));
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", global_img_depth).toImageMsg();
		depth_pub.publish(msg);
	
	}catch(cv_bridge::Exception& e){
		ROS_ERROR("%s", e.what());
		return;
	}
	
	// Process only on button click or something
	drawable = process_image(global_img_rgb, global_img_depth);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawable).toImageMsg();
	parsed_pub.publish(msg);

	
}


int main(int argc, char** argv){

    ros::init(argc, argv, "ros_capture");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

	depth_pub = it.advertise("/corkiris/depth", 1);
	parsed_pub = it.advertise("/corkiris/parsed", 1);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/camera/depth_registered/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camerainfo(n, "/camera/rgb/camera_info", 1);
    using MyPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
    MyPolicy mypolicy(32);

    
	message_filters::Synchronizer<MyPolicy> sync{static_cast<const MyPolicy &>(mypolicy), image_sub, depth_sub};
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
	ROS_INFO("Started Synchronizer\n");

    ros::spin();
    return 0;

}