//
// Created by liangdaxin on 23-7-12.
//

#include "../include/Hough.h"

Hough::Hough(cv::Mat img, int threshold) : img_(img), threshold_(threshold) {
    int height = img.rows;
    int width = img.cols;
    x_ = width;
    y_ = height;
    max_radius_ = int(sqrt(float(y_ * y_ + x_ * x_)));
    vote_matrix_ = cv::Mat::zeros(y_, x_, CV_32SC(max_radius_));

}

void Hough::get_angle() {
    cv::Mat img = img_;
    cv::Mat &angle = angle_;

    cv::Mat x_kernel = (cv::Mat_<int>(1, 2) << -1, 1);
    cv::Mat y_kernel = (cv::Mat_<int>(2, 1) << 1, -1);
    cv::Mat x_derivative = cv::Mat::zeros(img.size(), CV_32F);
    cv::Mat y_derivative = cv::Mat::zeros(img.size(), CV_32F);


    int height = img.rows;
    int width = img.cols;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (j == 0) {
                x_derivative.at<float>(i, j) = 1;
            } else {
                x_derivative.at<float>(i, j) = img.at<uint8_t>(i, j) * x_kernel.at<int>(0, 0) +
                                               img.at<uint8_t>(i, j + 1) * x_kernel.at<int>(0, 1);
            }
            if (i == 0) {
                y_derivative.at<float>(i, j) = 1;
            } else {
                y_derivative.at<float>(i, j) = img.at<uint8_t>(i, j) * y_kernel.at<int>(0, 0) +
                                               img.at<uint8_t>(i + 1, j) * y_kernel.at<int>(1, 0);
            }
        }
    }
    angle = cv::Mat::zeros(img.size(), CV_32F);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            float x_d = x_derivative.at<float>(i, j);
            float y_d = y_derivative.at<float>(i, j);
            auto a = sqrt(float(x_d * x_d + y_d * y_d));
            angle.at<float>(i, j) = atan2(float(y_d), float(x_d));
        }
    }
}

void Hough::Hough_transform() {
    /*
            按照 x,y,radius 建立三维空间，根据图片中边上的点沿梯度方向对空间中的所有单元进行投票。每个点投出来结果为一折线。
     */

    cv::Canny(img_, img_, 15, 45);

    cv::Mat img = img_;
    int height = img.rows;
    int width = img.cols;
    cv::Mat &vote_matrix = vote_matrix_;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (img.at<uint8_t>(i, j) > 0) {
                int x = j;
                int y = i;
                int radius = 0;
                float angle = angle_.at<float>(i, j);// -pi - pi

                //梯度方向在x正半轴
                if (abs(angle + M_PI) < 0.001 || abs(angle) < 0.001) {
                    int count = 0;

                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x+
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x += 1;
                        radius = floor(sqrt(pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                    x = j;
                    y = i;
                    radius = 0;
                    count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x-
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x -= 1;
                        radius = floor(sqrt(pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                }
                    //梯度方向在y正半轴
                else if (abs(M_PI / 2 - angle) < 0.001) {
                    int count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //y+
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        y += 1;
                        radius = floor(sqrt(pow(abs(y - i), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                    x = j;
                    y = i;
                    radius = 0;
                    count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //y-
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        y -= 1;
                        radius = floor(sqrt(pow(abs(y - i), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                }
                    //梯度方向在x负半轴
                else if (abs(M_PI - angle) < 0.001) {
                    int count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x-
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x -= 1;
                        radius = floor(sqrt(pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                    x = j;
                    y = i;
                    radius = 0;
                    count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x+
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x += 1;
                        radius = floor(sqrt(pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                }
                    //梯度方向在y负半轴
                else if (abs(M_PI / 2 + angle) < 0.001) {
                    int count = 0;
                    if(radius > 1000){
                        int a = 10;
                    }
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //y-
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        y -= 1;
                        radius = floor(sqrt(pow(abs(y - i), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                    x = j;
                    y = i;
                    radius = 0;
                    count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //y+
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        y += 1;
                        radius = floor(sqrt(pow(abs(y - i), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                }
                    //梯度方向在第一象限
                else if (angle > 0 && angle < M_PI / 2) {
                    int count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x+,y+
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x += 1;
                        float temp = tan(angle);
                        y += floor(temp);
                        radius = floor(sqrt(pow(abs(y - i), 2) + pow((x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }

                    x = j;
                    y = i;
                    radius = 0;
                    count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x-,y-
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x -= 1;
                        float temp = tan(angle);
                        y -= floor(temp);
                        radius = floor(sqrt(pow(abs(y - i), 2) + pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                }

                    //梯度方向在第二象限
                else if (angle > M_PI / 2 && angle < M_PI) {
                    angle -= M_PI / 2;
                    int count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x-,y+
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x -= 1;
                        float temp = tan(angle);
                        y += floor(temp);
                        radius = floor(sqrt(pow(abs(y - i), 2) + pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                    x = j;
                    y = i;
                    radius = 0;
                    count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x+,y-
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x += 1;
                        float temp = tan(angle);
                        y -= floor(temp);
                        radius = floor(sqrt(pow(abs(y - i), 2) + pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                }

                    //梯度方向在第三象限
                else if (angle < -M_PI / 2 && angle > -M_PI) {
                    angle += M_PI;
                    int count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x-,y-
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x -= 1;
                        float temp = tan(angle);
                        y -= floor(temp);
                        radius = floor(sqrt(pow(abs(y - i), 2) + pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                    x = j;
                    y = i;
                    radius = 0;
                    count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x+,y+
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x += 1;
                        float temp = tan(angle);
                        y += floor(temp);
                        radius = floor(sqrt(pow(abs(y - i), 2) + pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                }
                    //梯度方向在第四象限
                else if (angle > -M_PI / 2 && angle < 0) {
                    angle = -angle;
                    int count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x+,y-
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x += 1;
                        float temp = tan(angle);
                        y -= floor(temp);
                        radius = floor(sqrt(pow(abs(y - i), 2) + pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                    x = j;
                    y = i;
                    radius = 0;
                    count = 0;
                    while (count < 70 && x >= 0 && y >= 0 && x < width && y < height) {
                        //x-,y+
                        if(radius > 1000){
                            int a = 10;
                        }
                        vote_matrix.at<cv::Vec3i>(y, x)[radius]++;
                        x -= 1;
                        float temp = tan(angle);
                        y += floor(temp);
                        radius = floor(sqrt(pow(abs(y - i), 2) + pow(abs(x - j), 2)));
                        count++;
                        if(radius > 200){
                            break;
                        }
                    }
                }
            }
        }
    }
}

void Hough::Select_Circle() {
    int x = x_;
    int y = y_;
    int r = max_radius_;
    int threshold = threshold_;
    int max = 0;
    int arr[200] = {0};
    const cv::Mat &vote_matrix = vote_matrix_;
    for (int i = 0; i < y; i++) {
        for (int j = 0; j < x; j++) {
            for (int k = 0; k < r; k++) {
                max = max > vote_matrix.at<cv::Vec3i>(i, j)[k] ? max : vote_matrix.at<cv::Vec3i>(i, j)[k];
                if (vote_matrix.at<cv::Vec3i>(i, j)[k] != 0) {
                    arr[vote_matrix.at<cv::Vec3i>(i, j)[k]]++;
                }
                if (vote_matrix.at<cv::Vec3i>(i, j)[k] == 13) {
                    std::cout << i << " " << j << " " << k << std::endl;
                }
//                if (vote_matrix.at<cv::Vec3i>(i, j)[k] > threshold) {
//
//                }
            }
        }
    }
    std::cout << max << std::endl;
}