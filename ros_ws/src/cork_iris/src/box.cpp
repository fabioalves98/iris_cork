#include "box.h"

void Box::removeBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, 
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
                    cv::Mat cv_image)
{
    std::vector<cv::Point> contours = getCorkContours(cv_image);

    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    copyPointCloud(*cloud_in, *cloud_out);

    for (int x = 0; x < cloud_in->width; x++)
    {
        for (int y = 0; y < cloud_in->height; y++)
        {
            if (cv::pointPolygonTest(contours, cv::Point(x, y), false) != 1)
            {
                cloud_out->at(x, y).x = cloud_out->at(x, y).y = cloud_out->at(x, y).z = bad_point;
            }
        }
    }
}

std::vector<cv::Point> Box::getCorkContours(cv::Mat cv_image)
{        
    // Image box
    cv_image = get_blue_box(cv_image);
    
    // Get blue box painted all blue points bright red. Get the mask for all bright red points
    cv::Mat mask = getMaskInRange(cv_image, cv::Scalar(0, 0, 250), cv::Scalar(0, 0, 255));

    // Contours
    int min_area = 20000;
    int max_area = 200000;

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat hierarchy;

    cv::findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    hierarchy.release();

    std::vector<std::vector<cv::Point>> filtered_contours;

    for( int i = 0; i < contours.size(); i++ )
    {
        int area = cv::contourArea(contours[i]);
        if(area < max_area && area > min_area)
        {
            filtered_contours.push_back(contours[i]);
        }
    }

    min_area = cv::contourArea(filtered_contours[0]);
    int idx = 0;

    for( int i = 0; i < filtered_contours.size(); i++ )
    {
        int area = cv::contourArea(filtered_contours[i]);
        if(area < min_area)
        {
            min_area = area;
            idx = i;
        }
    }

    return filtered_contours[idx];
}

cv::Mat Box::get_blue_box(cv::Mat cv_image)
{
    for (int i = 0; i < cv_image.rows; i++)
    {
        for (int j = 0; j < cv_image.cols; j++)
        {
            cv::Vec3b pixel = cv_image.at<cv::Vec3b>(i,j);
            int r = static_cast<int>(pixel.val[2]);
            int g = static_cast<int>(pixel.val[1]);
            int b = static_cast<int>(pixel.val[0]);

            if ((b > g) && (g > r + 10) && (b > 50)) 
            {
                circle(cv_image, cv::Point(j, i), 0, cv::Scalar(0, 0, 255));
            }
        }
    } 

    return cv_image;
}

cv::Mat Box::getMaskInRange(cv::Mat cv_image, cv::Scalar min, cv::Scalar max)
{
    cv::Mat mask;   
    cv::inRange(cv_image, min, max, mask);
    return mask;
}