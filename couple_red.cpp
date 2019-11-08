#include "couple_red.h"

Find_couple_red::Find_couple_red(){
    cout << "is ready!!" << endl;
}

void Find_couple_red::run_Main(Mat & srcImg){
    cout << "the pic has putout!!" << endl;
    imshow("srcImg", srcImg);

    vector<cv::Mat> bgr;
    split(srcImg, bgr);
    Mat result_img;
    if(color_ == 0){
        subtract(bgr[2], bgr[1], result_img);
    }
    else{
        subtract(bgr[0], bgr[2], result_img);
    }
    Mat threImg;
    threshold(result_img, threImg, 50, 255, 0);
    imshow("threImg", threImg);
    Mat whiteImg(srcImg.size(), srcImg.type(), Scalar(0));

    vector<RotatedRect> led;
    vector<vector<Point>> contour_light;
    vector<Vec4i> hierarchy;
    findContours(threImg, contour_light, hierarchy,2,CHAIN_APPROX_NONE);
    for(int i1 = 0; i1 < (int)contour_light.size(); i1++){
        //有父,子轮廓，不满住6点拟合椭圆的先排除
        if(hierarchy[i1][3]>0 || hierarchy[i1][2]>0 || contour_light[i1].size() < 6)
            continue;

        double area = contourArea(contour_light[i1]);
        //double length = arcLength(contour_light[i1], true);
        //轮廓面积过小或者过大，排除
        if(area < 80 || area > 1e4)
            continue;

        //Rect rect = boundingRect(contour_light[i1]);
        //rectangle(whiteImg, rect, Scalar(0,0,255), 2, LINE_8, 0);

        RotatedRect fit_ellipse = fitEllipse(contour_light[i1]);

        //归一化椭圆角度
        if(fit_ellipse.angle > 90.0f)
            fit_ellipse.angle = fit_ellipse.angle - 180.0f;
        //限制角度,宽高比
        if(fabs(fit_ellipse.angle) <= 40 &&
                (double)fit_ellipse.boundingRect().width/(double)fit_ellipse.boundingRect().height < 0.5){
            led.push_back(fit_ellipse);
            ellipse(whiteImg, fit_ellipse, Scalar(0,0,255), 2, 8);
            //cout << "size:" << led.size() << endl;
        }
    }
    //排除符合轮廓数量少于2
    if(led.size() < 2)
        return;

    vector<RotatedRect> couple_leds(2);
    vector<vector<RotatedRect>> fit_led;
    for(int i = 0; i < led.size(); i++){
        for(int j = i+1; j < led.size(); j++){
            //筛选高度,椭圆角度相似度。若条件符合，作为一对。
            if(is_Horizon(led[i].center, led[j].center) < 0.2 && is_Aline(led[i], led[j]) < 10.0f){
                if(led[i].center.x < led[j].center.x){
                    couple_leds[0] = led[i];
                    couple_leds[1] = led[j];
                }
                else {
                    couple_leds[1] = led[i];
                    couple_leds[0] = led[j];
                }
                fit_led.push_back(couple_leds);
                cout << "num:" << fit_led.size() << endl;
            }
        }
    }

    double last_area;
    double max_area;
    double average_led;
    vector<RotatedRect> final_couple(2);
    vector<vector<RotatedRect>> final_led;
    for(int k = 0; k < fit_led.size(); k++){
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

        average_led = (max_lenth_led + min_lenth_led)/2;
        double dist_led = is_Dist_led(a_pair_l.center, a_pair_r.center);
        //固定尺寸比例
        if(dist_led > 3.0f*average_led || dist_led < 1.5*average_led)
            continue;
        cout << "dist:" << dist_led << "  average:" << average_led << endl;

        double area_led = average_led * dist_led;
        max_area = (area_led > last_area ? area_led : last_area);
        last_area = area_led;
        //面积最大
        if(max_area == area_led){
            final_couple[0] = a_pair_l;
            final_couple[1] = a_pair_r;
            final_led.push_back(final_couple);
        }
        line(whiteImg, final_couple[0].center, final_couple[1].center, Scalar(0,255,0), 2, 4, 0);
    }

    Mat roi_rect = threImg;
    if(final_led.size() > 0){
        Point2f center_rect((final_couple[0].center.x +final_couple[1].center.x)/2, (final_couple[0].center.y +final_couple[1].center.y)/2);
        double up_limit_y = (center_rect.y - 3.0f*average_led < 0 ? 0: center_rect.y - 3.0f*average_led);
        roi_rect = threImg(Range(up_limit_y, center_rect.y), Range(final_couple[0].center.x, final_couple[1].center.x));
    }

    line(whiteImg, final_couple[0].center, final_couple[1].center, Scalar(0,0,255), 2, 4, 0);
    imshow("roi", roi_rect);
    imshow("output", whiteImg);
}

double Find_couple_red::is_Horizon(Point2f & center_i, Point2f & center_j){
    double gradient = fabs((center_i.y - center_j.y)/(center_i.x - center_j.x));
    cout << "gradient:" << gradient << endl;
    return gradient;
}

double Find_couple_red::is_Aline(RotatedRect & rect_led_i, RotatedRect & rect_led_j){
    double dif_angle;

    if((double)rect_led_i.boundingRect().width < (double)rect_led_i.boundingRect().height){
        if((double)rect_led_j.boundingRect().width < (double)rect_led_j.boundingRect().height){//左左
            dif_angle = fabs(rect_led_i.angle - rect_led_j.angle);
        }
        else {//左右
            dif_angle = fabs((90.0f - rect_led_i.angle) - rect_led_j.angle);
        }
    }
    else {
        if((double)rect_led_j.boundingRect().width > (double)rect_led_j.boundingRect().height){//右右
            dif_angle = fabs(rect_led_i.angle - rect_led_j.angle);
        }
        else {//右左
            dif_angle = fabs((90.0f - rect_led_j.angle) - rect_led_i.angle);
        }
    }

    return dif_angle;
}

double Find_couple_red::is_Dist_led(Point2f & point_i, Point2f & point_j){

    double dist = pow(pow(point_i.x - point_j.x, 2) + pow(point_i.y - point_j.y, 2), 0.5);

    return dist;
}

Find_couple_red::~Find_couple_red(){
    ;
}
