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
    void image_Processing(Mat & src);
    void find_Contours(Mat & roi);
    void choose_Target(bool & is_find, RotatedRect & left, RotatedRect & right, double & d, double & a);
    Rect get_Roi(bool is_target, Mat img, RotatedRect l, RotatedRect r, double dis, double aver, Mat & src);

    double is_Horizon(Point2f center_i, Point2f center_j);
    double is_Aline(RotatedRect rect_led_i, RotatedRect rect_led_j);
    double is_Dist_led(Point2f point_i, Point2f point_j);
    RotatedRect boundRect(RotatedRect L_led, RotatedRect R_led, double W, double H);
    void make_RotatedSafe(Mat bound_size);


    int color_ = 0;

    RotatedRect bound_rotated;
    Point2f center_point;
    Rect last_roi;
    bool succe_match = false;
    bool is_run = false;
    int lost_cnt;
    Mat result_img;
    vector<vector<RotatedRect>> fit_led;

};



#endif // COUPLE_RED_H
