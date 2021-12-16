#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "armor_detect.h"

using namespace cv;
using namespace std;

int main()
{
    Mat video1, new_image;
    int g_nContrastValue=100;//100
    int g_nBrightValue=-100;
    VideoCapture video("der.avi");//打开视频
	namedWindow("识别窗口", CV_WINDOW_AUTOSIZE);
    while(1)
    {
        video.read(video1);
        new_image=video1.clone();
        for(int y = 0; y < video1.rows; y++ )
        {
            for(int x = 0; x < video1.cols; x++ )
            {
                for(int c = 0; c < 3; c++ )
                {
                    new_image.at<Vec3b>(y,x)[c]= saturate_cast<uchar>( (g_nContrastValue*0.01)*(video1.at<Vec3b>(y,x)[c] ) + g_nBrightValue );
                }
            }
        }
        imshow("ho",new_image);
        ArmorDetector armordetector;
        armor arm;
        Rect roi = armordetector.GetRoi(new_image);//
        armordetector.DetectArmor(new_image,roi);//
        imshow("detect", new_image);
        if(waitKey(1)==27) break;
    }
    return 0;
}