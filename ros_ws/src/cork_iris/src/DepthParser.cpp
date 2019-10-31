#include "DepthParser.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>

DepthParser::DepthParser(void)
{
 
}

DepthParser::~DepthParser(void)
{

}


void DepthParser::extendDepthImageColors(cv::Mat image, std::vector<cv::Point> contour)
{
    cv::Point highest = DepthParser::findMinMaxPoint(image, contour, true); 
    cv::Point lowest  = DepthParser::findMinMaxPoint(image, contour, false);
    std::cout << lowest << " " << highest << std::endl;
    
    int i, j; 
    unsigned char *ptr = (unsigned char*)(image.data);
    int minval = ptr[image.step * highest.y + highest.x];
    int maxval = ptr[image.step * lowest.y + lowest.x]; 
    int diff = abs(maxval-minval);

    for(i = 0; i < image.cols; i++){
        for(j = 0; j < image.rows; j++){
            if(ptr[image.cols * j + i] < minval){
                ptr[image.cols * j + i] = 0;
            }else if(ptr[image.cols * j + i] >= maxval){
                ptr[image.cols * j + i] = 255;           
            }else{
                ptr[image.cols * j + i] = (ptr[image.cols * j + i] - minval) * (255 / diff);
   
            }
        }
    }
    
}

cv::Point DepthParser::findMinMaxPoint(cv::Mat image, std::vector<cv::Point> contour, bool toggle)
{
    int min = 256;
    int max = -1;
    int x = 0, y = 0;
    unsigned char *input = (unsigned char*)(image.data);
    for(int j = 0; j < image.rows;j++){
        for(int i = 0; i < image.cols;i++){
            unsigned char b = input[image.step * j + i ] ;
            unsigned char g = input[image.step * j + i + 1];
            unsigned char r = input[image.step * j + i + 2];

            // why do some pixels have different values rgb values? (0, 255, 0) ... etc
            if(r == g && r == b){
                if(toggle){
                    if(r < min && r != 0){
                        int isInside = cv::pointPolygonTest(contour, cv::Point(i, j), false);
                        if(isInside == 1)
                        {
                            min = r;
                            x = i;
                            y = j;
                        }
                    }
                }else{
                    if(r > max && r != 255){
                        int isInside = cv::pointPolygonTest(contour, cv::Point(i, j), false);
                        if(isInside == 1)
                        {
                            max = r;
                            x = i;
                            y = j;
                        }
                    }
                }
                    
            }

        }
    }
    return cv::Point(x, y);

}

std::vector<cv::Point> DepthParser::getPointNeighbours(cv::Point p)
{
    std::vector<cv::Point> neighbours_norm = {cv::Point(-1,-1), cv::Point(-1, 0), cv::Point(-1, 1),
                                cv::Point(0,-1), cv::Point(0,0), cv::Point(0, 1),
                                cv::Point(1, -1), cv::Point(1, 0), cv::Point(1,1)};

    std::vector<cv::Point> neighbours;
    for(int i = 0; i < neighbours_norm.size(); i++){
        neighbours.push_back(p + neighbours_norm.at(i));
    }

    return neighbours;
}



cv::Mat DepthParser::getBestPossibleCorkPiece(cv::Mat input_image, std::vector<cv::Point> contour)
{
    cv::Point highest = DepthParser::findMinMaxPoint(input_image, contour, true);
    
    std::vector<cv::Point> pixels = getPointNeighbours(highest);
    cv::Mat output_image = input_image.clone();

    unsigned char *input = (unsigned char*)(input_image.data);
    unsigned char *output = (unsigned char*)(output_image.data);

    const int BLACK_THRESHOLD = 120;

    int MAX_ITERS = 20000;
    for(int i = 0; i < pixels.size(); i++){
        if(MAX_ITERS == 0) break;

        cv::Point p = pixels.at(i);
        int pcolor = (int)input[input_image.step * p.y + p.x + 2];   
        std::vector<cv::Point> neighbours = DepthParser::getPointNeighbours(p);
        for(int j = 0; j < neighbours.size(); j++){
            cv::Point pneighbour = neighbours.at(j);
            int ncolor = (int)input[input_image.step * pneighbour.y + pneighbour.x + 2]; 

            if(!(find(pixels.begin(), pixels.end(), pneighbour) != pixels.end()))
            {
                if(abs(pcolor-ncolor) < 9 && pcolor <= ncolor)
                    pixels.push_back(pneighbour);
            
                else
                {
                    output[output_image.step * pneighbour.y + pneighbour.x + 2] = 0;   
                    output[output_image.step * pneighbour.y + pneighbour.x + 1] = 0;   
                    output[output_image.step * pneighbour.y + pneighbour.x ] = 255;   
                }
            }

                
            
        }
        MAX_ITERS--;
    }

    return output_image;
}




