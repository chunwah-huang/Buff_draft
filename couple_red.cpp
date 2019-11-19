#include "couple_red.h"

Find_couple_red::Find_couple_red(){
    //cout << "is ready!!" << endl;
}

void Find_couple_red::run_Main(Mat & srcImg){

    Mat roi_img;
    image_Processing(srcImg);
    if(is_run){
        result_img(last_roi).copyTo(roi_img);
    }
    else {
        result_img.copyTo(roi_img);
    }

    find_Contours(roi_img);
    RotatedRect L, R;
    double average_led;
    double dist_led;
    succe_match = false;
    choose_Target(succe_match, L, R, dist_led, average_led);
    get_Roi(succe_match, result_img, L, R, dist_led, average_led, srcImg);

    rectangle(srcImg, last_roi, Scalar(0, 255, 0), 2, 8, 0);
    fit_led.clear();

    imshow("roi", roi_img);
    imshow("output", srcImg);
}

void Find_couple_red::image_Processing(Mat & src){
    vector<Mat> bgr;
    split(src, bgr);
    if(color_ == 0){
        subtract(bgr[2], bgr[1], result_img);
    }
    else{
        subtract(bgr[0], bgr[2], result_img);
    }
    threshold(result_img, result_img, 25, 255, 0);
    //imshow("inputImg", result_img);
}

void Find_couple_red::find_Contours(Mat & roi){
    vector<RotatedRect> led;
    vector<vector<Point>> contour_light;
    vector<Vec4i> hierarchy;
    findContours(roi, contour_light, hierarchy,2,CHAIN_APPROX_NONE);
    for(int i1 = 0; i1 < (int)contour_light.size(); ++i1){
        //有父,子轮廓，不满住6点拟合椭圆的先排除
        if(hierarchy[i1][3]>0 || hierarchy[i1][2]>0 || contour_light[i1].size() < 6)
            continue;

        double area = contourArea(contour_light[i1]);
        //double length = arcLength(contour_light[i1], true);
        //轮廓面积过小或者过大，排除
        if(area < 80 || area > 1e4)
            continue;

        RotatedRect fit_ellipse = fitEllipse(contour_light[i1]);

        //归一化椭圆角度
        if(fit_ellipse.angle > 90.0f)
            fit_ellipse.angle = fit_ellipse.angle - 180.0f;
        //限制角度,宽高比
        if(fabs(fit_ellipse.angle) <= 40.0f &&
                (float)fit_ellipse.boundingRect().width/(float)fit_ellipse.boundingRect().height < 0.4f){
            led.push_back(fit_ellipse);
            //ellipse(srcImg, fit_ellipse, Scalar(0,0,255), 2, 8);
        }
    }

    //排除符合轮廓数量少于2
    if(led.size() >= 2){
        vector<RotatedRect> couple_leds(2);
        for(int i = 0; i < (int)led.size(); ++i){
            for(int j = i+1; j < (int)led.size(); ++j){
                //筛选高度,椭圆角度相似度。若条件符合，作为一对。
                if(is_Horizon(led[i].center, led[j].center) < 0.3 && is_Aline(led[i], led[j]) < 20.0f){
                    if(led[i].center.x < led[j].center.x){
                        couple_leds[0] = led[i];
                        couple_leds[1] = led[j];
                    }
                    else {
                        couple_leds[1] = led[i];
                        couple_leds[0] = led[j];
                    }
                    fit_led.push_back(couple_leds);
                    //cout << "num:" << match_size.size() << endl;
                }
            }
        }
    }
}

void Find_couple_red::choose_Target(bool & is_find, RotatedRect & left, RotatedRect & right, double & d, double & a){
    if(fit_led.size() > 0){
        //cout << "in" << endl;
        double last_area;
        double max_area;
        for(int k = 0; k < (int)fit_led.size(); k++){
            RotatedRect a_pair_l(fit_led[k][0]);
            RotatedRect a_pair_r(fit_led[k][1]);
            //line(whiteImg, a_pair_l.center, a_pair_r.center, Scalar(0,255,0), 2, 4, 0);

            double height_led_l = max(a_pair_l.boundingRect().height, a_pair_l.boundingRect().width);
            double height_led_r = max(a_pair_r.boundingRect().height, a_pair_r.boundingRect().width);
            double max_lenth_led, min_lenth_led;
            if(height_led_l > height_led_r){
                max_lenth_led = height_led_l;
                min_lenth_led = height_led_r;
            }
            else {
                max_lenth_led = height_led_r;
                min_lenth_led = height_led_l;
            }
            //灯条相似度
            if(min_lenth_led < 0.7*max_lenth_led)
                continue;

            a = (max_lenth_led + min_lenth_led)/2;
            d = is_Dist_led(a_pair_l.center, a_pair_r.center);
            //固定尺寸比例
            if(d > 5.0f*a || d < 1.5f*a)
                continue;
            //cout << "dist:" << dist_led << "  average:" << average_led << endl;

            double area_led = a * d;
            max_area = (area_led > last_area ? area_led : last_area);
            last_area = area_led;
            //面积最大
            if(max_area == area_led){
                left = a_pair_l;
                right = a_pair_r;
                is_find = true;
            }
        }
    }
}

double Find_couple_red::is_Horizon(Point2f center_i, Point2f center_j){
    double gradient = fabs((center_i.y - center_j.y)/(center_i.x - center_j.x));
    //cout << "gradient:" << gradient << endl;
    return gradient;
}

double Find_couple_red::is_Aline(RotatedRect rect_led_i, RotatedRect rect_led_j){
    double dif_angle;

    if((float)rect_led_i.boundingRect().width < (float)rect_led_i.boundingRect().height){
        if((float)rect_led_j.boundingRect().width < (float)rect_led_j.boundingRect().height){//左左
            dif_angle = fabs(rect_led_i.angle - rect_led_j.angle);
        }
        else {//左右
            dif_angle = fabs((90.0 - rect_led_i.angle) - rect_led_j.angle);
        }
    }
    else {
        if((float)rect_led_j.boundingRect().width > (float)rect_led_j.boundingRect().height){//右右
            dif_angle = fabs(rect_led_i.angle - rect_led_j.angle);
        }
        else {//右左
            dif_angle = fabs((90.0 - rect_led_j.angle) - rect_led_i.angle);
        }
    }

    return dif_angle;
}

double Find_couple_red::is_Dist_led(Point2f point_i, Point2f point_j){

    double dist = pow(pow(point_i.x - point_j.x, 2) + pow(point_i.y - point_j.y, 2), 0.5);

    return dist;
}

RotatedRect Find_couple_red::boundRect(RotatedRect L_led, RotatedRect R_led, double W, double H){

    float angle = atan2(R_led.center.y - L_led.center.y, R_led.center.x - L_led.center.x);

    Point2f center((L_led.center.x +R_led.center.x)/2, (L_led.center.y +R_led.center.y)/2);
    //cout << "angle:" << angle * 180 / CV_PI << endl;
    return RotatedRect(center, Size2f(W, H), angle * 180 / CV_PI);
}

Rect Find_couple_red::get_Roi(bool is_target, Mat img, RotatedRect l, RotatedRect r, double dis, double aver, Mat & src){
    if(is_target){
        lost_cnt = 0;
        //识别坐标并转换
        center_point.x = (l.center.x + r.center.x)/2 + last_roi.x;
        center_point.y = (l.center.y + r.center.y)/2 + last_roi.y;
        //保留last_Rotated
        bound_rotated = boundRect(l, r, dis, aver);
        circle(src, center_point, 3, Scalar(0, 255, 0), 3, 8, 0);
    }
    else {
        if(bound_rotated.size.width > 0 || bound_rotated.size.height > 0){
            ++lost_cnt;
            bool is_increase = false;
            if(lost_cnt < 3)
                bound_rotated.size = Size2f(bound_rotated.size.width, bound_rotated.size.height);
            else if(lost_cnt == 3){
                bound_rotated.size = Size2f(bound_rotated.size.width*1.2, bound_rotated.size.height*1.2);
                is_increase = true;
            }
            else if(lost_cnt == 5){
                bound_rotated.size = Size2f(bound_rotated.size.width*1.2, bound_rotated.size.height*1.2);
                is_increase = true;
            }
            else if(lost_cnt == 7){
                bound_rotated.size = Size2f(bound_rotated.size.width*1.2, bound_rotated.size.height*1.2);
                is_increase = true;
            }
            else if(lost_cnt == 9){
                bound_rotated.size = Size2f(bound_rotated.size.width*1.5, bound_rotated.size.height*1.5);
                is_increase = true;
            }
            else if(lost_cnt > 11){
                bound_rotated = RotatedRect();
                lost_cnt = 0;
            }

            if(is_increase){
                make_RotatedSafe(img);
            }
        }
        else {
            bound_rotated = RotatedRect();
        }
    }

    if(bound_rotated.size.width > 0 || bound_rotated.size.height > 0){
        Rect rect = bound_rotated.boundingRect();
        int W, H;
        W = rect.width*1.5;
        H = rect.height*4;
        int x, y;
        x = MAX(center_point.x-W*0.5, 0);
        y = MAX(center_point.y-H*0.5, 0);
        Point TL = Point(x, y);
        x = MIN(center_point.x+W*0.5, img.cols);
        y = MIN(center_point.y+H*0.5, img.rows);
        Point BR = Point(x, y);
        last_roi = Rect(TL, BR);
    }
    else {
        last_roi = Rect(0, 0, img.cols, img.rows);
    }
    is_run = true;
    return last_roi;
}

void Find_couple_red::make_RotatedSafe(Mat bound_size){
    Point2f vertex[4];
    bound_rotated.points(vertex);
    sort(vertex, vertex + 4, [](const Point2f & p1, const Point2f & p2) {return p1.x < p2.x;});
    if(vertex[0].x < 0 || vertex[4].x > bound_size.cols)
        bound_rotated = RotatedRect();
    sort(vertex, vertex + 4, [](const Point2f & p3, const Point2f & p4) {return p3.y < p4.y;});
    if(vertex[0].y < 0 || vertex[4].y > bound_size.rows)
        bound_rotated = RotatedRect();
}

Find_couple_red::~Find_couple_red(){
    ;
}
