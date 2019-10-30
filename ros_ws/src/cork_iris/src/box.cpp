#include "box.h"

#include <stdlib.h>

Box::Box(void)
{
 
}

std::vector<cv::Point> Box::get_pins(cv::Mat image)
{
    std::vector<cv::Point> possible_pins;

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            cv::Vec3b pixel = image.at<cv::Vec3b>(i,j);
            int r = static_cast<int>(pixel.val[2]);
            int g = static_cast<int>(pixel.val[1]);
            int b = static_cast<int>(pixel.val[0]);

            if ((r > 200) && (g < 100) && (b < 150)) 
            {
                possible_pins.push_back(cv::Point(j, i));
            }
        }
    }

    std::vector<cv::Point> good_pins;

    for (int i = 0; i < possible_pins.size(); i++)
    {
        cv::Point candidate = possible_pins.at(i);
        bool already_good = false;
        for (int k = 0; k < good_pins.size(); k++)
        {
            if (cv::norm(cv::Mat(candidate), cv::Mat(good_pins.at(k))) < 10 )
            {
                already_good = true;
            }
        }
        if (already_good) continue;

        int count = 0;
        for (int j = 0; j < possible_pins.size(); j++)
        {
            cv::Point comp = possible_pins.at(j);

            if (cv::norm(cv::Mat(candidate), cv::Mat(comp)) < 10)
            {
                count ++;
            }
            if (count > 5)
            {
                good_pins.push_back(candidate);
                break;
            }
        }
    }

    return good_pins;
}

void Box::draw_rect(cv::Mat image, std::vector<cv::Point> pins)
{
    cv::Point min = cv::Point(640, 480);
    cv::Point max = cv::Point(0, 0);

    for (int i = 0; i < pins.size(); i++)
    {
        if (min.x + min.y > pins[i].x + pins[i].y)
        {
            min = pins[i];
        }
        if (max.x + max.y < pins[i].x + pins[i].y)
        {
            max = pins[i];
        }
    }
    for (int i = 0; i < pins.size(); i++)
    {
        if (pins[i] == min || pins[i] == max)
        {
            continue;
        }
        else
        {
            line(image, min, pins[i], cv::Scalar(0, 0, 255), 2);
            line(image, max, pins[i], cv::Scalar(0, 0, 255), 2);
        }   
    }
}

std::vector<cv::Point> Box::get_blue_box(cv::Mat image)
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

            if ((b > g) && (g > r + 10) && (b > 80)) 
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

cv::Mat Box::getMaskInRange(cv::Mat image, cv::Scalar min, cv::Scalar max)
{
    cv::Mat mask;   
    cv::inRange(image, min, max, mask);
    return mask;
}