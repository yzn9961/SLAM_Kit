#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;
int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "ERROR：输入格式错误！ 正确格式：./可执行文件 输入视频路径" << endl;
        return 1;
    }
    VideoCapture cap(argv[1]);
    int Frame_w = cap.get(CAP_PROP_FRAME_WIDTH);
    int Frame_h = cap.get(CAP_PROP_FRAME_HEIGHT);
    long frameToStart = 50;//给双目三秒钟的时间初始化
    cap.set(CAP_PROP_POS_FRAMES, frameToStart);
    // 初始化视频写入器
    Mat src;
    cap >> src;
    bool isColor = (src.type() == CV_8UC3);
    VideoWriter Leftwriter;
    VideoWriter Rightwriter;
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    string Leftname = "./left.avi";
    string Rightname = "./right.avi";
    double fps = 25.0;
    Leftwriter.open(Leftname, codec, fps, Size((int)Frame_w/2,Frame_h), isColor);//写入左侧
    Rightwriter.open(Rightname, codec, fps, Size((int)Frame_w/2,Frame_h), isColor);//写入右侧
    Rect Lrect(0,0, Frame_w/2,Frame_h);//左ROI
    Rect Rrect(Frame_w/2, 0, Frame_w/2, Frame_h); //右ROI
    //视频畸变处理环节
    //读取畸变矫正参数
        cv::FileStorage fsSettings("./my2cam.yaml", cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: 参数路径错误！" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: 参数不完整！" << endl;
            return -1;
        }
        cout<<"相机内参读取完成！"<<endl;
        // 计算畸变矫正参数
           cv::Mat M1l,M2l,M1r,M2r;
           cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
           cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
           cout<<"畸变矫正参数计算完成！"<<endl;

    while (cap.isOpened()) {
        cap >> src;
        cv::Mat imLeftRect, imRightRect;
        if(src.empty())
        {
            cout<<"视频处理结束！"<<endl;
            break;
        }
        cv::remap(src(Lrect),imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(src(Rrect),imRightRect,M1r,M2r,cv::INTER_LINEAR);
        Leftwriter.write(imLeftRect);
        Rightwriter.write(imRightRect);
        imshow("On progress",src);
        if (waitKey(30) >= 0)
            break;

    }
    return 0;

}
