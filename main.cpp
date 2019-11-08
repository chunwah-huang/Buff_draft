#include <iostream>
#include <opencv2/opencv.hpp>
#include "couple_red.h"
using namespace std;
using namespace cv;

int main()
{
    //"/home/hzh/图片/2020-能量机关.png"
    Mat src = imread("/home/hzh/图片/2020-buff(复件).png", 1);
    //imshow("src", src);
    //resize(src, src, Size(640, 480));
    Find_couple_red RM_dist;
    RM_dist.run_Main(src);

//    VideoCapture cap;
//    cap.open(1);

//    if(!cap.isOpened())
//        cout << "can not open the cap!" << endl;
//    return -1;

//    Find_couple_red RM_dist;
//    Mat frame;
//    while(1){
//        cap >> frame;
//        imshow("frame", frame);
//        resize(frame, frame, Size(640, 480));
//        RM_dist.run_Main(frame);

//        char c = waitKey(1);
//        if(c == 27){
//            break;
//        }
//    }
    waitKey(0);
    return 0;
}
