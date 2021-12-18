/****************************************************************************
 *  Copyright (C) 2019 cz.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include "armor_detect.h"
//#include <opencv2/opencv.hpp>

void White_equal(const Mat whi1,Mat whi2);
// 计算两个点之间的距离
double calc_distance(Point2f p1, Point2f p2)
{
    return pow(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2),0.5);
}

armor::armor(){}

armor::armor(const LED_Stick& L1, const LED_Stick& L2)
{
    Led_stick[0]= L1;
    Led_stick[1]= L2;
    error_angle = fabs(L1.rect.angle - L2.rect.angle);//求对浮点型绝对值，头文件是#include <math.h>

    rect.width = abs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));//static_cast强制转换成int类型  | abs()对整型求绝对值；
    rect.height = static_cast<int>((L1.rect.size.height + L2.rect.size.height)/2);
    center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x)/2);
    center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y)/2);
    rect.x = center.x - rect.width/3;                //？
    rect.y = center.y - rect.height/3;               //？
    rect.width *= 2.0/3;//rect.width=rect.width* 2/3
    rect.height *= 2.0/3;
}

void armor::draw_rect( Mat& img, Point2f roi_offset_point) const  //const在函数名后，只能发生读操作,不能发生写操作
{
    rectangle(img, rect + Point_<int>(roi_offset_point), Scalar(255,255,255), 1);//白色
}

void armor::draw_spot(Mat &img, Point2f roi_offset_point) const
{
    circle(img, center + Point_<int>(roi_offset_point), int(rect.height/4), Scalar(0,0,255), -1);//蓝色
}


int armor::get_average_intensity(const Mat& img) //计算装甲板roi平均色彩强度，用于筛选装甲板中心有灯条
{
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
    // namedWindow("roi",WINDOW_NORMAL);
    // imshow("roi",roi);
    average_intensity = static_cast<int>(mean(roi).val[0]);//mean()求平均值；
    return average_intensity;
}


void armor::max_match(vector<LED_Stick>& LED,size_t i,size_t j)
{
    RotatedRect R, L;
    //RotatedRect类中包含外接矩形的中心center、大小size以及角度angle   
    //Point2f center; //矩形的质心
    //Size2f size;   //矩形的边长
    //float angle; 
    if(Led_stick[0].rect.center.x > Led_stick[1].rect.center.x)//判断左右灯条；
    {
        R = Led_stick[0].rect;
        L = Led_stick[1].rect;
    }
    else
    {
        R = Led_stick[1].rect;
        L = Led_stick[0].rect;
    }

    float angle_8 = L.angle - R.angle;//两个灯条的误差的角度
    //    cout << L.angle << " "<< R.angle << endl;
    if(angle_8 < 1e-3f)
        angle_8 = 0.0f;
    float f = error_angle + 0.5 * angle_8;
    if(!LED.at(i).matched && !LED.at(j).matched )
    {
        LED.at(i).matched = true;
        LED.at(i).match_index = j;//j=1
        LED.at(j).matched = true;
        LED.at(j).match_index = i;
        LED.at(i).match_factor = f;
        LED.at(j).match_factor = f;
    }
    if(LED.at(i).matched && !LED.at(j).matched)
    {
        if(f < LED.at(i).match_factor)//i j重新配对   //f越小越好
        {
            LED.at(LED.at(i).match_index).matched = false;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
            LED.at(j).matched = true;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;

        }
    }
    if(LED.at(j).matched && !LED.at(i).matched)
    {
        if(f < LED.at(j).match_factor )
        {
            LED.at(LED.at(j).match_index).matched = false;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;
            LED.at(i).matched = true;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
        }
    }
    if(LED.at(j).matched && LED.at(i).matched
            && LED.at(i).match_factor > f && LED.at(j).match_factor > f)
    {
        LED.at(LED.at(j).match_index).matched = false;
        LED.at(LED.at(i).match_index).matched = false;
        LED.at(i).matched = true;
        LED.at(i).match_factor = f;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_factor = f;
        LED.at(j).match_index = i;
    }
}

bool armor::is_suitable_size(void) const
{
    // 两个灯条体型相似
    if(Led_stick[0].rect.size.height*0.7f < Led_stick[1].rect.size.height && Led_stick[0].rect.size.height*1.3f > Led_stick[1].rect.size.height)
    {
        float armor_width = fabs(Led_stick[0].rect.center.x - Led_stick[1].rect.center.x);
        
        if(armor_width > Led_stick[0].rect.size.width && armor_width > Led_stick[1].rect.size.width &&
             armor_width > (Led_stick[0].rect.size.width+Led_stick[1].rect.size.width)*3)
        {
            float h_max = (Led_stick[0].rect.size.height + Led_stick[1].rect.size.height)/2.0f;
            // 两个灯条高度差不大
            if(fabs(Led_stick[0].rect.center.y - Led_stick[1].rect.center.y) < 0.8f* h_max )
            {
                // 长宽比判断
                if(h_max*4.0f > rect.width && h_max < 1.2f* rect.width)// 5/6 <  x/y  < 4
                {
                    return true;
                }
            }
        }
    }
    return false;
}


Rect ArmorDetector::GetRoi(const Mat &img)
{
    Size img_size = img.size();         //？
    Rect rect_tmp = last_target_;       //上一次框的目标；
    Rect rect_roi;
    if(rect_tmp.x == 0 || rect_tmp.y == 0 || rect_tmp.width == 0 || rect_tmp.height == 0 || lost_cnt_ >= 15 || detect_cnt_%100 == 0)
    {
        last_target_ = Rect(0,0,img_size.width, img_size.height);//Rect（左上角x坐标  ，  左上角y坐标，矩形的宽，矩形的高）
        rect_roi = Rect(0,0,img_size.width, img_size.height);//Roi等于整个图片；
        return rect_roi;
    }
    else
    {
        float scale = 2;
        if (lost_cnt_ < 30)
            scale = 3;
        else if(lost_cnt_ <= 60)
            scale = 4;
        else if(lost_cnt_ <= 120)
            scale = 5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width)*0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height)*0.5f);

        rect_roi = Rect(x, y, w, h);

        if(makeRectSafe(rect_roi, img_size)== false)
        {
            rect_roi = Rect(0,0,img_size.width, img_size.height);
        }
    }
    return rect_roi;
}

bool ArmorDetector::DetectArmor(Mat &img, Rect roi_rect)
{
    Mat roi_image = img(roi_rect);
    Point2f offset_roi_point (roi_rect.x, roi_rect.y);
    vector<LED_Stick> LED_Stick_v;  // 声明所有可能的灯条容器

    Mat binary_brightness_img;
    Mat binary_color_img;
    Mat gray;
    Mat result_img;

    cvtColor(roi_image,gray,COLOR_BGR2GRAY);//转为灰度图
    threshold(gray, binary_brightness_img, 50, 255, THRESH_BINARY);//对灰度图进行二值化
    imshow("binary_brightness_img", binary_brightness_img);

    vector<cv::Mat> bgr;
    split(roi_image, bgr);

    if(color_ == 0)//
    {
        subtract(bgr[2], bgr[0], result_img);//通道相减
    }
    else
    {
        subtract(bgr[0], bgr[2], result_img);
    }

    threshold(result_img, binary_color_img, 25, 255, THRESH_BINARY);//对通道相减后的图像进行二值化
    imshow("binary_color_img1", binary_color_img);
    
    vector<vector<Point>> contours_light;
    vector<vector<Point>> contours_brightness;

    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);//检测通道相减后的二值化图像的最外层轮廓+所有轮廓点
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);//检测灰度图的二值化图像的最外层轮廓+所有轮廓点
    
    //画出装甲板
    for(size_t i = 0; i < contours_brightness.size(); i++)//遍历灰度图的二值化图像的轮廓
    {
        double area = contourArea(contours_brightness[i]);
        if (area < 20.0 || area > 1e5 ) continue;

        for(size_t ii = 0; ii < contours_light.size(); ii++)//遍历通道相减后的二值化图像的轮廓
        {
            if(pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0 )//判断第一个点在不在轮廓上
            {
                double length = arcLength(contours_brightness[i], true); // 灯条周长（灰度图的轮廓）【封闭】
                if (length > 15 && length <4000)
                {   
                    // fitEllipse()输出中心点坐标，以及矩形的长度和宽度还有矩形的偏转角度                 
                    RotatedRect RRect = fitEllipse(contours_brightness[i]);        // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
                    // 旋转矩形提取四个点
                    Point2f rect_point[4];
                    RRect.points(rect_point);           //返回矩形的4个顶点
                    // for (int i = 0; i < 4 ; i++)
                    // {
                    //     line(img, rect_point[i]+offset_roi_point, rect_point[(i+1)%4]+offset_roi_point, Scalar(255,0,255),1);
                    // }
                    // 角度换算，将拟合椭圆0~360 -> -180~180
                    if(RRect.angle>90.0f)                                  //？
                        RRect.angle =  RRect.angle - 180.0f;               //？
                    // putText(img, to_string(RRect.angle), RRect.center + Point2f(2,2) + offset_roi_point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
                    //to_string：将数值转化为字符串，返回对应的字符串
                    if (fabs(RRect.angle) <= 30)  // 超过一定角度的灯条不要
                    {
                        LED_Stick r(RRect);
                        LED_Stick_v.push_back(r);
                    }
                }
                break;
            }
        }
    }

    // **寻找可能的装甲板** -遍历每个可能的灯条, 两两灯条拟合成装甲板进行逻辑判断
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        for(size_t j = i + 1; j < LED_Stick_v.size() ; j++)
        {
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(j) );
            if (arm_tmp.error_angle < 8.0f)
            {
                // putText(img, to_string(arm_tmp.rect.width/(arm_tmp.rect.height+0.0001)), arm_tmp.center + Point_<int>(offset_roi_point) , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
                // TODO(cz): 推荐加入灯条宽度要小于装甲板宽度的条件
                if(arm_tmp.is_suitable_size())
                {
                    // TODO(cz): 推荐使用255值的面积进行判断
                    if(arm_tmp.get_average_intensity(gray)< 50 )
                    {
                        arm_tmp.max_match(LED_Stick_v, i, j);
                    }
                }
            }
        }
    }

    // **分类装甲板** -根据灯条匹配状态得到最终装甲板
    vector<armor> final_armor_list;
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        if(LED_Stick_v.at(i).matched)
        {
            LED_Stick_v.at(LED_Stick_v.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(LED_Stick_v.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }

    // **选择装甲板** -根据距离图像中心最短选择
    float dist=1e8;
    bool found_flag = false;
    armor target;
    Point2f roi_center(roi_rect.width/2, roi_rect.height/2);
    float dx,dy;
    for (size_t i = 0; i < final_armor_list.size() ; i++ )
    {
#ifdef FAST_DISTANCE
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
#else
        dx = pow((final_armor_list.at(i).center.x - roi_center.x), 2.0f);
        dy = pow((final_armor_list.at(i).center.y - roi_center.y), 2.0f);
#endif
        if( dx + dy < dist)
        {
            target = final_armor_list.at(i);
            dist = dx + dy;
        }
#ifdef SHOW_DRAW_RECT
        final_armor_list.at(i).draw_rect(img, offset_roi_point);
#endif
        found_flag = true;
    }
#ifdef SHOW_ROI_RECTANGLE
    rectangle(img, roi_rect,Scalar(255, 0, 255),1);
#endif
    // **计算装甲板四个点顶点** -用于pnp姿态结算
    // TODO(cz): 四个点的不同的bug修复
    RotatedRect target_rect;
    if(found_flag)
    {
#ifdef SHOW_DRAW_SPOT
        target.draw_spot(img, offset_roi_point);
#endif
        Point2f point_tmp[4];
        Point2f point_2d[4];
        // 左右灯条分类，本别提取装甲板四个外角点
        RotatedRect R, L;
        if(target.Led_stick[0].rect.center.x > target.Led_stick[1].rect.center.x)
        {
            R = target.Led_stick[0].rect;
            L = target.Led_stick[1].rect;
        }else
        {
            R = target.Led_stick[1].rect;
            L = target.Led_stick[0].rect;
        }
        L.points(point_tmp);
        point_2d[0] = point_tmp[1];
        point_2d[3] = point_tmp[0];
        R.points(point_tmp);
        point_2d[1] = point_tmp[2];
        point_2d[2] = point_tmp[3];
        // 计算补偿，用于调试调整准心
        Point2f offset_point;
        if(cap_mode_ == 0)
        {
            offset_point = Point2f(100, 100) - Point2f(short_offset_x_,short_offset_y_);
        }
        else
        {
            offset_point = Point2f(100, 100) - Point2f(long_offset_x_,long_offset_y_);
        }

        points_2d_.clear();
        vector<Point2f> points_roi_tmp;
        for(int i=0;i<4;i++)
        {
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
            points_2d_.push_back(point_2d[i] + offset_roi_point +offset_point);
//            putText(img, to_string(i), points_2d_.at(i), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
//            circle(img, points_2d_.at(i), 5, Scalar(255, 255, 255), -1);
//            circle(img, points_2d_.at(i), 3, Scalar(i*50, i*50, 255), -1);
        }
        // 计算当前装甲板类型，到后面task中还有滤波，可以有误差
        // float armor_h = target.rect.height;
        // float armor_w = target.rect.width;
        // if(armor_w / armor_h < 3.3f)
        //     is_small_ = 1;
        // else
        //     is_small_ = 0;

        //计算ROI的相关参数
        last_target_ = boundingRect(points_roi_tmp);
        rectangle(img, last_target_,Scalar(255,255,255), 1);
        lost_cnt_ = 0;
    }else {
        //计算ROI的相关参数
        lost_cnt_ ++;
    }
    detect_cnt_++;
    return found_flag;
}
void brightAdjust(Mat src,Mat dst,double dContrast,double dBright)
{
    int nVal;
    for(int nI = 0;nI<src.rows;nI++)
    {
        Vec3b* p1 = src.ptr<Vec3b>(nI);
        Vec3b* p2 = dst.ptr<Vec3b>(nI);
        for(int nJ =0;nJ<src.cols;nJ++)
        {
            for(int nK =0;nK<3;nK++)
            {
                nVal = (int)(dContrast *p1[nJ][nK] + dBright);
                if(nVal<0)
                    nVal=0;
                if(nVal>255)
                    nVal=255;
                p2[nJ][nK] = nVal;
            }
        }
    }
}

void White_equal(const Mat whi1,Mat whi2)
{
	//Mat imageSource = imread("02.jpg");
	//imshow("原始图像", imageSource);
	vector<Mat> imageRGB;
 
	//RGB三通道分离
	split(whi1, imageRGB);
 
	//求原始图像的RGB分量的均值
	double R, G, B;
	B = mean(imageRGB[0])[0];
	G = mean(imageRGB[1])[0];
	R = mean(imageRGB[2])[0];
 
	//需要调整的RGB分量的增益
	double KR, KG, KB;
	KB = (R + G + B) / (3 * B);
	KG = (R + G + B) / (3 * G);
	KR = (R + G + B) / (3 * R);
 
	//调整RGB三个通道各自的值
	imageRGB[0] = imageRGB[0] * KB;
	imageRGB[1] = imageRGB[1] * KG;
	imageRGB[2] = imageRGB[2] * KR;
 
	//RGB三通道图像合并
	merge(imageRGB, whi2);
	imshow("白平衡调整后", whi2);
}

int main( int argc, char** argv )
{
    VideoCapture capture;// /home/lishengnan/A1/B1/ap01.avi
    capture.open("/home/lishengnan/A1/B1/ap01.avi");
    Mat image;
    
while(1)
    {
        capture >> image;//读入帧
        brightAdjust(image,image,1,-150);
        // White_equal(image,image);
        ArmorDetector detector;
        Rect roi = detector.GetRoi(image);
        detector.DetectArmor(image,roi);
        imshow("video",image);
        waitKey(1);
    }
}