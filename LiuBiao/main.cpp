#include <iostream>
#include <opencv2/opencv.hpp>
#include "armor_detect.cpp"

using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap;
    cap.open("/home/liubiao/files/video/armor3.avi");
    if(!cap.isOpened())
    {
        cout << "open failed" << endl;
        return 0;
    }

    Mat frame1;
    while(true)
    {
        cap.read(frame1);
        if(frame1.empty())
        {
            cout << "frame over" << endl;
            break;
        }
        //Mat frame = frame1(Rect(0, 0, 570, 500));
        float t0 = getTickCount();

        LED_Stick le;
        armor arm;
        ArmorDetector arm_det;
        
        bright_adjust(frame1);
        Mat frame = white_balance(frame1);
        Size _size_ = frame.size();
        Rect _rect_ = arm_det.GetRoi(frame);
        bool a = arm_det.makeRectSafe( _rect_, _size_);
        arm_det.DetectArmor(frame, _rect_);

        float t1 = getTickCount();
        float t = (t1 - t0);
        float fps = getTickFrequency() / t;
        putText(frame, to_string((int)fps), Point(12, 50), cv::FONT_HERSHEY_COMPLEX, 2, Scalar(255, 0, 0), 2);

        namedWindow("dst_", WINDOW_AUTOSIZE);
        imshow("dst_", frame);
        waitKey(1);
    }
    cap.release();
    return 0;
}