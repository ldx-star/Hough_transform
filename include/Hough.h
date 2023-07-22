//
// Created by liangdaxin on 23-7-12.
//

#ifndef HOUGH_TRANSFORM_HOUGH_H
#define HOUGH_TRANSFORM_HOUGH_H

#include <opencv2/opencv.hpp>

class Hough {
public:
    struct Circle {
        int x;
        int y;
        float radius;
    };

public:
    int x_;
    int y_;
    int max_radius_;
    cv::Mat vote_matrix_;
    cv::Mat img_;
    Circle circle_;
    int threshold_;
private:
    cv::Mat angle_;

public:
    Hough(cv::Mat img,int threshold);

    void Hough_transform();

    void get_angle();

    void Select_Circle();


};


#endif //HOUGH_TRANSFORM_HOUGH_H
