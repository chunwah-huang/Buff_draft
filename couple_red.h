#ifndef COUPLE_RED_H
#define COUPLE_RED_H

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class Find_couple_red{
public:
    Find_couple_red();
    ~Find_couple_red();
    /*--------*/
    void run_Main(Mat & srcImg);


private:
    void remove_Angle(RotatedRect & fit_rect, vector<RotatedRect> & led_1);
    double is_Horizon(Point2f & center_i, Point2f & center_j);
    double is_Aline(RotatedRect & rect_led_i, RotatedRect & rect_led_j);
    double is_Dist_led(Point2f & point_i, Point2f & point_j);

    int roi_choose = 0;
    enum roi_choose{
        success = 1,
        lose = 2,
    };

    int color_ = 0;

};



#endif // COUPLE_RED_H
