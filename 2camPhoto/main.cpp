/*
使用双目摄像头"HBV-1780-2 S2.0"进行拍照
画幅tl点x坐标:0 y坐标：150
画幅宽度：640 画幅高度：180
*/
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture cap(2);
    if (!cap.isOpened())
    {
        cout << "摄像头读取错误!" << endl;
        return 1;
    }
    //获取视频尺寸
    char filename[200];
    //cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    //cap.set(CAP_PROP_FRAME_HEIGHT,480);
    //cap.set(CAP_PROP_FRAME_WIDTH, 640);
    //cap.set(CAP_PROP_FRAME_HEIGHT,480);
    int i =1;
    string name;
    cout << "拍照模式：按空格键进行拍摄。q 或 Esc 退出。" << endl;
    string window_name = "video | q or esc to quit";
    namedWindow(window_name,WINDOW_NORMAL); //not resizable window;
    Mat frameTotal;
    Rect Lrect(0,0, 1280/2,480);//左ROI
    Rect Rrect(1280 / 2, 0, 1280/2, 480); //右ROI
    while (1)
    {
        cap >> frameTotal;
        imshow("video | q or esc to quit", frameTotal);
        char key = (char)waitKey(30);
        switch (key)
        {
        case'q':
        case'Q':
        case 27:
            return 0;
        case ' ':
            sprintf(filename, "./data_new/raw/Capture_%d.jpg", i);
            imwrite(filename, frameTotal);//保存全画幅
            sprintf(filename, "./data_new/left/L_%d.jpg", i);
            imwrite(filename, frameTotal(Lrect));//保存左画幅
            sprintf(filename, "./data_new/right/R_%d.jpg", i);
            imwrite(filename, frameTotal(Rrect));//保存右画幅
            cout << "捕捉到第"<<i<<"组图像" << endl;
            i++;
        default:
            break;
        }

    }
    return 0;
}
