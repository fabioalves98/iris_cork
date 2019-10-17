#include "box.h"

#include <stdlib.h>

using namespace cv;

Box::Box(void)
{
 
}

std::vector<cv::Point> Box::get_pins(cv::Mat image)
{
    std::vector<Point> possible_pins;

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            Vec3b pixel = image.at<cv::Vec3b>(i,j);
            int r = static_cast<int>(pixel.val[2]);
            int g = static_cast<int>(pixel.val[1]);
            int b = static_cast<int>(pixel.val[0]);

            if ((r > 200) && (g < 100) && (b < 150)) 
            {
                possible_pins.push_back(Point(j, i));
            }
        }
    }

    std::vector<Point> good_pins;

    for (int i = 0; i < possible_pins.size(); i++)
    {
        Point candidate = possible_pins.at(i);
        bool already_good = false;
        for (int k = 0; k < good_pins.size(); k++)
        {
            if (norm(Mat(candidate), Mat(good_pins.at(k))) < 10 )
            {
                already_good = true;
            }
        }
        if (already_good) continue;

        int count = 0;
        for (int j = 0; j < possible_pins.size(); j++)
        {
            Point comp = possible_pins.at(j);

            if (norm(Mat(candidate), Mat(comp)) < 10)
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
    Point min = Point(640, 480);
    Point max = Point(0, 0);

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
            line(image, min, pins[i], Scalar(0, 0, 255), 2);
            line(image, max, pins[i], Scalar(0, 0, 255), 2);
        }   
    }
}