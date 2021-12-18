#pragma once 

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/**
 * @brief 装甲板灯条相关数据信息
 */
class LED_Stick{
public:

    LED_Stick():matched(false){}

    LED_Stick(const RotatedRect& R)
    {
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }
    RotatedRect rect;   // 装甲板灯条相关数据信息
    bool matched;       // 匹配状态， 用于灯条匹配
    size_t match_index; // 匹配对应的灯条序号， 用于灯条匹配
    float match_factor; // 匹配强度， 用于灯条匹配
};

/**
 * @brief 装甲板相关数据信息
 */
class armor{
public:
    armor();
    armor(const LED_Stick& L1, const LED_Stick& L2);

    void draw_rect( Mat& img, Point2f roi_offset_poin) const;    // 画出装甲板
    void draw_spot(Mat &img, Point2f roi_offset_point) const;    // 画中心点
    int get_average_intensity(const Mat& img) ; // 计算装甲板roi平均色彩强度，用于筛选装甲板中心有灯条
    void max_match(vector<LED_Stick>& LED, size_t i, size_t j); // 灯条匹配算法
    bool is_suitable_size(void) const;          // 判断可能的装甲板是否符合尺寸

    LED_Stick Led_stick[2];  // 装甲板的两个灯条
    float error_angle;       // 两个灯条的误差的角度
    Point2i center;          // 装甲板中心点
    Rect2i rect;             // 装甲板roi矩形
    int average_intensity;   // 装甲板roi的平均色彩强度
};

struct OtherParam
{
    int color = 1;       // 我方车辆颜色，0是蓝色，1是红色。用于图像预处理
    int mode = 0;        // 视觉模式，0是自瞄模式，1是能量机关模式
    int cap_mode = 1;    // 摄像头类型，0是短焦摄像头，1是长焦摄像头
    float gimbal_data;
    float buff_offset_x;
    float buff_offset_y;
};

/**
 * @brief 装甲板检测器
 */
class ArmorDetector
{
public:
    ArmorDetector(){}
    ~ArmorDetector(){}
    int ArmorDetectTask(Mat &img, OtherParam other_param);
    void DrawTarget(Mat &img)
    {
        if(!points_2d_.empty())
        {
            line(img, points_2d_[0],points_2d_[2],Scalar(255,100,0), 3);
            line(img, points_2d_[1],points_2d_[3],Scalar(255,100,0), 3);
        }
    }
    void getAngle(float &yaw, float &pitch)
    {
        yaw = angle_x_;
        pitch = angle_y_;
    }

    /**
     * @brief GetRoi 获取图像ROI区域
     * @param img
     * @return 返回感兴趣区域的矩形
     */
    Rect GetRoi(Mat &img);
    bool makeRectSafe(Rect & rect,Size size){
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }
    bool DetectArmor(Mat &img,Rect roi_rect);
    bool getTypeResult(bool is_small);
    void setFilter(int filter_size){
        filter_size_ = filter_size;
    }
    void clear(){
        history_.clear();
    }
    // 调试参数
    int short_offset_x_ = 100;
    int short_offset_y_ = 100;
    int long_offset_x_ = 100;
    int long_offset_y_ = 100;

    /**
     * @brief DetectArmor 装甲板识别函数
     * @param img
     * @param roi_rect
     * @return
     */
    // 外部参数
    int color_=0;
    int cap_mode_;
    // ｒｏｉ参数
    Rect last_target_;
    int lost_cnt_ = 0;
    int detect_cnt_ = 0;

    float dist_ = 3000; // 通过距离更换相机
    float r_=0.5; // 距离刷新率 (0-1)
    int update_cap_cnt = 0; // 用于强制限制相机更新频率
    float angle_x_ = 0;
    float angle_y_ = 0;
    float distance_ = 0;
    vector<Point2f> points_2d_;

    // 判断大小装甲板类型相关参数
    std::list<bool> history_;
    int filter_size_ = 5;
    bool is_small_;


};