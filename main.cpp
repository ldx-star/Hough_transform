//
// Created by liangdaxin on 23-7-12.
//
#include <iostream>
#include <string>
#include "include/Hough.h"



int main(){
    std::string img_path = "../img/1.bmp";
    cv::Mat img = cv::imread(img_path,0);
    assert(img.data);
    Hough hough(img,135);
    hough.get_angle();
    hough.Hough_transform();
    hough.Select_Circle();
    return 0;
}