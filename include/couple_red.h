#ifndef COUPLE_RED_H
#define COUPLE_RED_H

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

struct match_target
{
    RotatedRect rect;

};

class Find_couple_red{

public:
    Find_couple_red();
    ~Find_couple_red();
    void run_Main(Mat & srcImg);

private:
    void image_Processing(Mat & src, Mat & binImg);
    void find_Contours(Mat & roi);
    void choose_Target(bool & is_find, RotatedRect & left, RotatedRect & right, double & d, double & a);
    Rect get_Roi(const bool & is_target, const Mat & img, const RotatedRect & l, const RotatedRect & r, const double & dis, const double & aver, Mat & src);

    double is_Horizon(const Point2f & center_i, const Point2f & center_j);
    double is_Aline(const RotatedRect & rect_led_i, const RotatedRect & rect_led_j);
    double is_Dist_led(const Point2f & point_i, const Point2f & point_j);
    RotatedRect boundRect(const RotatedRect & L_led, const RotatedRect & R_led, const double & W, const double & H);
    void make_RotatedSafe(const Mat & bound_size);

    void reSet(){
        bound_rotated = RotatedRect();
        lost_cnt = 0;
    }

    void dataClear(){
        fit_led.clear();
    }

private:
    int color_ = 0;
    RotatedRect bound_rotated;
    Point2f center_point;
    Rect last_roi;
    bool succe_match = false;
    bool is_run = false;
    int lost_cnt;
    vector<vector<RotatedRect>> fit_led;
};



#endif // COUPLE_RED_H
