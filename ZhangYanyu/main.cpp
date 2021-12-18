
#include "armor_detect.cpp"



int main(){
    Mat frame;
    ArmorDetector check;

    VideoCapture cap(0);
    // VideoCapture cap;
    // cap.open("/home/lx/ZhangYanyu/myarmor.mp4");

    if(!cap.isOpened()){
        cout<<"could not open camera...\n"<<endl;
        return -1;
    }

    namedWindow("armor_f", WINDOW_AUTOSIZE);
    namedWindow("effect",WINDOW_AUTOSIZE);
    while(1){
        cap>>frame;
        if(frame.empty()){
            break;
        }
    
        imshow("armor_f",frame);
        
        Rect roi_rect = check.GetRoi(frame);
        check.DetectArmor(frame, roi_rect);
        if((waitKey(40)==27))break;
    }
    return 0;
}