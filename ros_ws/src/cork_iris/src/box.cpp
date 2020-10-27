#include "box.h"

#include <stdlib.h>

Box::Box(cv::Mat image)
{
    this->image = image;
}

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
    cv::Mat box_image =  cv_image.clone();
    Box box(box_image);
    std::vector<cv::Point> points = box.get_blue_box();
    // Get blue box painted all blue points bright red. Get the mask for all bright red points
    cv::Mat mask = box.getMaskInRange(cv::Scalar(0, 0, 250), cv::Scalar(0, 0, 255));

    // Contours
    int min_area = 20000;
    int max_area = 200000;
    std::vector<std::vector<cv::Point>> contour_points;
    ImageParser ip(mask);

    // This contour should be the inside contour (excluding the all the box around the cork pieces)
    // This is a heavy assumption since we are considering that only two contours exist after the first
    // area filter, the outer box cntour and the inner box contour.
    contour_points.push_back(ip.smallestAreaContour(ip.filterContoursByArea(ip.parseImageContours(-1), min_area, max_area)));
    
    return contour_points.at(0);
}

std::vector<cv::Point> Box::get_blue_box()
{
    std::vector<cv::Point> box;

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            cv::Vec3b pixel = image.at<cv::Vec3b>(i,j);
            int r = static_cast<int>(pixel.val[2]);
            int g = static_cast<int>(pixel.val[1]);
            int b = static_cast<int>(pixel.val[0]);

            if ((b > g) && (g > r + 10) && (b > 50)) 
            {
                box.push_back(cv::Point(j, i));
                circle(image, cv::Point(j, i), 0, cv::Scalar(0, 0, 255));

            }
        }
    } 
    return box;
}

std::vector<cv::Point> Box::get_box_corners(std::vector<cv::Point> box_contour)
{

    std::vector<cv::Point> corners;
    cv::Point upper_left(640, 480);
    cv::Point upper_right(0, 1);
    cv::Point lower_left(1, 0);
    cv::Point lower_right(0, 0);

    for (int i = 0; i < box_contour.size(); i++)
    {
        cv::Point point = box_contour.at(i);
        // Upper Left
        if (upper_left.x + upper_left.y > point.x + point.y)
        {
            upper_left = point;
        }
        // Upper Rigth
        if (point.x > point.y && (abs(point.x - point.y) > abs(upper_right.x - upper_right.y)))
        {
            upper_right = point;
        }
        // Lower Left
        if (point.x < point.y && (abs(point.x - point.y) > abs(lower_left.x - lower_left.y)))
        {
            lower_left = point;
        }
        // Lower right
        if (lower_right.x + lower_right.y < point.x + point.y)
        {
            lower_right = point;
        }
    }
    corners.push_back(upper_left);
    corners.push_back(upper_right);
    corners.push_back(lower_left);
    corners.push_back(lower_right);

    return corners;
}

cv::Mat Box::getMaskInRange(cv::Scalar min, cv::Scalar max)
{
    cv::Mat mask;   
    cv::inRange(image, min, max, mask);
    return mask;
}