#include "box.h"

#include <stdlib.h>

Box::Box(cv::Mat image)
{
    this->image = image;
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