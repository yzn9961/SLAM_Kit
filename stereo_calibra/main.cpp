#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

//定义标定板内角点维数
int CHESSBOARD[2] = {6,9};//纵向6个角点，横向9个角点
using namespace std;
using namespace cv;

Rect validROIL, validROIR;
Mat R, T, E, F; //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
Mat Rl, Rr, Pl, Pr, Q;//校正旋转矩阵R，投影矩阵P 重投影矩阵Q (下面有具体的含义解释）
cv::Mat R_cameraMatrix, R_distCoeffs, R_R, R_T;
cv::Mat L_cameraMatrix, L_distCoeffs, L_R, L_T;
Mat R_src,R_img,L_src,L_img,R_dst,L_dst;//图像
double fx;

Mat show_stereoCalib() //展示双目立体标定结果
{
    Mat mapLx, mapLy, mapRx, mapRy;
    Mat srcL = imread("./data_new/left/L_1.jpg",COLOR_BGR2GRAY);// 左目畸变图像
    Mat srcR = imread("./data_new/right/R_1.jpg",COLOR_BGR2GRAY);// 右目畸变图像
    cv::Size imageSize(cv::Size(srcR.cols, srcR.rows));
    initUndistortRectifyMap(L_cameraMatrix, L_distCoeffs, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(R_cameraMatrix, R_distCoeffs, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
    Mat rectifyImageL,rectifyImageR;
    remap(srcL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    remap(srcR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

    Mat canvas;
    double sf;
    int w, h;
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);
    //左图像画到画布上
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分
    resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小
    Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域
        cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
    //rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形
    //cout << "Painted ImageL" << endl;

    //右图像画到画布上
    canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分
    resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
        cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
    //rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
    //cout << "Painted ImageR" << endl;

    //画上对应的线条
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    //imshow("rectified", canvas);
    return canvas;
}


Mat handwrite_Undistort(Mat distort)//手写畸变校正
{
    Mat img_undistort(distort.rows, distort.cols,CV_8UC3);//ȥ��������ɫͼ��
    double cx,cy,fx,fy,p1,p2,k1,k2,k3;
    cx = Pl.at<double>(0,2);
    cy = Pl.at<double>(1,2);
    fx = Pl.at<double>(0,0);
    fy = Pl.at<double>(1,1);
    p1 = L_distCoeffs.at<double>(0,2);
    p2 = L_distCoeffs.at<double>(0,3);
    k1 = L_distCoeffs.at<double>(0,0);
    k2 = L_distCoeffs.at<double>(0,1);
    k3 = L_distCoeffs.at<double>(0,4);
    for (int v = 0; v < distort.rows; v++)
    {
        for (int u = 0; u < distort.cols; u++)
        {
            double x = (u - cx) / fx, y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);

            double x_distorted = x * (1 + k1 * pow(r, 2) + k2 * pow(r, 4) + k3 * pow(r, 6)) + 2 * p1*x*y + p2*(pow(r, 2) + 2 * pow(x, 2));
            double y_distorted = y * (1 + k1 * pow(r, 2) + k2 * pow(r, 4) + k3 * pow(r, 6)) + p1 * (pow(r, 2) + 2 * pow(y, 2)) + 2 * p2*x*y;

            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;

            if (u_distorted >= 0 &&
                v_distorted >= 0 &&
                u_distorted < distort.cols&&
                v_distorted < distort.rows)
            {
                for(int i=0;i<3;i++)
                    img_undistort.at<Vec3b>(v, u)[i] = distort.at<Vec3b>((int)v_distorted, (int)u_distorted)[i];
            }
            else
            {
                for (int i = 0; i < 3; i++)
                    img_undistort.at<Vec3b>(v, u)[i] = 0;
            }
        }
    }
    return img_undistort;
}
void outputCameraParam(void)
{
    /*保存数据*/
    /*输出数据*/
    fx = Pl.at<double>(0,0);
    FileStorage fs("intrinsics.yml", FileStorage::WRITE);  //文件存储器的初始化
    if (fs.isOpened())
    {
        fs << "L_cameraMatrix" << L_cameraMatrix << "L_cameraDistcoeffs" << L_distCoeffs << "R_cameraMatrix" << R_cameraMatrix << "R_cameraDistcoeffs" << R_distCoeffs;
        fs.release();
        //cout << "cameraMatrixL=:" << L_cameraMatrix << endl << "cameraDistcoeffL=:" << R_distCoeffs << endl << "cameraMatrixR=:" << R_cameraMatrix << endl << "cameraDistcoeffR=:" << R_distCoeffs << endl;
        //右摄像头参数输出
        cout << "/////////////////// 右摄像头内参 ///////////////////" << endl;
        std::cout << "cameraMatrix : " << R_cameraMatrix << std::endl;
        std::cout << "distCoeffs : " << R_distCoeffs << std::endl << endl;
        //左摄像头参数输出
        cout << "/////////////////// 左摄像头内参 ///////////////////" << endl;
        std::cout << "cameraMatrix : " << L_cameraMatrix << std::endl;
        std::cout << "distCoeffs : " << L_distCoeffs << std::endl << endl << endl;
    }
    else
    {
        cout << "Error: 无法保存内参!" << endl;
    }


    fs.open("extrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        cout << "/////////////////// 双目立体视觉标定外参 ///////////////////" << endl;
        fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
        cout << "R=" << R << endl << "T=" << T << endl << "Rl=" << Rl << endl << "Rr=" << Rr << endl << "Pl=" << Pl << endl << "Pr=" << Pr << endl << "Q=" << Q << endl;
        fs.release();
    }
    else
        cout << "Error: 无法保存外参！\n";

    // 写入双目orb-slam3参数格式
    fs.open("my2cam.yaml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        // 公共参数
        //fs<<""<<;
        fs<<"Camera_type"<<"\"PinHole\"";
        fs<<"Camera_fx"<<Pl.at<double>(0,0);//.at()先行后列 (00)
        fs<<"Camera_fy"<<Pl.at<double>(1,1);//11
        fs<<"Camera_cx"<<Pl.at<double>(0,2);//02
        fs<<"Camera_cy"<<Pl.at<double>(1,2);//12

        fs<<"Camera_k1"<<L_distCoeffs.at<double>(0,0);//00
        fs<<"Camera_k2"<<L_distCoeffs.at<double>(0,1);//01
        fs<<"Camera_p1"<<L_distCoeffs.at<double>(0,2);//02
        fs<<"Camera_p2"<<L_distCoeffs.at<double>(0,3);//03

        fs<<"Camera_bFishEye"<<0;

        fs<<"Camera_width"<<R_img.cols;
        fs<<"Camera_height"<<R_img.rows;

        fs<<"Camera_fps"<<30;
        fs<<"Camera_bf"<<0.06*fx;//基线长度
        fs<<"Camera_RGB"<<1;
        fs<<"ThDepth"<<35.0;

        // 左侧相机参数
        fs<<"LEFT_width"<<L_img.cols;
        fs<<"LEFT_height"<<L_img.rows;
        fs<<"LEFT_D"<<L_distCoeffs;//左相机原始畸变矩阵
        fs<<"LEFT_K"<<L_cameraMatrix;//左相机原始内参矩阵
        fs<<"LEFT_R"<<Rl;//左相机极线矫正矩阵
        fs<<"LEFT_P"<<Pl;//左相机极线矫正后的投影矩阵
        // 右侧相机参数
        fs<<"RIGHT_width"<<R_img.cols;
        fs<<"RIGHT_height"<<R_img.rows;
        fs<<"RIGHT_D"<<R_distCoeffs;//左相机原始畸变矩阵
        fs<<"RIGHT_K"<<R_cameraMatrix;//左相机原始内参矩阵
        fs<<"RIGHT_R"<<Rr;//左相机极线矫正矩阵
        fs<<"RIGHT_P"<<Pr;//左相机极线矫正后的投影矩阵
        // ORB特征设置
        fs<<"ORBextractor_nFeatures"<<1200;
        fs<<"ORBextractor_scaleFactor"<<1.2;
        fs<<"ORBextractor_nLevels"<<8;
        fs<<"ORBextractor_iniThFAST"<<20;
        fs<<"ORBextractor_minThFAST"<<7;
        // 视角设置
        fs<<"Viewer_KeyFrameSize"<<0.05;
        fs<<"Viewer_KeyFrameLineWidth"<<1;
        fs<<"Viewer_GraphLineWidth"<<0.9;
        fs<<"Viewer_PointSize"<<2;
        fs<<"Viewer_CameraSize"<<0.08;
        fs<<"Viewer_CameraLineWidth"<<3;
        fs<<"Viewer_ViewpointX"<<0;
        fs<<"Viewer_ViewpointY"<<-0.7;
        fs<<"Viewer_ViewpointZ"<<-1.8;
        fs<<"Viewer_ViewpointF"<<500;

        fs.release();
    }
    else
        cout << "Error: 无法保存为ORB标准格式！\n";
    // 输出单目orb-slam3参数格式
    fs.open("my1cam.yaml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        // 公共参数
        fs<<"Camera_type"<<"\"PinHole\"";
        fs<<"Camera_fx"<<Pl.at<double>(0,0);//.at()先行后列 (00)
        fs<<"Camera_fy"<<Pl.at<double>(1,1);//11
        fs<<"Camera_cx"<<Pl.at<double>(0,2);//02
        fs<<"Camera_cy"<<Pl.at<double>(1,2);//12

        fs<<"Camera_k1"<<L_distCoeffs.at<double>(0,0);//00
        fs<<"Camera_k2"<<L_distCoeffs.at<double>(0,1);//01
        fs<<"Camera_p1"<<L_distCoeffs.at<double>(0,2);//02
        fs<<"Camera_p2"<<L_distCoeffs.at<double>(0,3);//03

        fs<<"Camera_bFishEye"<<0;

        fs<<"Camera_width"<<R_img.cols;
        fs<<"Camera_height"<<R_img.rows;

        fs<<"Camera_fps"<<30;
        fs<<"Camera_RGB"<<1;
        fs<<"ThDepth"<<35.0;
        // ORB特征设置
        fs<<"ORBextractor_nFeatures"<<1200;
        fs<<"ORBextractor_scaleFactor"<<1.2;
        fs<<"ORBextractor_nLevels"<<8;
        fs<<"ORBextractor_iniThFAST"<<20;
        fs<<"ORBextractor_minThFAST"<<7;
        // 视角设置
        fs<<"Viewer_KeyFrameSize"<<0.05;
        fs<<"Viewer_KeyFrameLineWidth"<<1;
        fs<<"Viewer_GraphLineWidth"<<0.9;
        fs<<"Viewer_PointSize"<<2;
        fs<<"Viewer_CameraSize"<<0.08;
        fs<<"Viewer_CameraLineWidth"<<3;
        fs<<"Viewer_ViewpointX"<<0;
        fs<<"Viewer_ViewpointY"<<-0.7;
        fs<<"Viewer_ViewpointZ"<<-1.8;
        fs<<"Viewer_ViewpointF"<<500;

        fs.release();
    }
    else
        cout << "Error: 无法保存为ORB标准格式！\n";
    cout<<"参数已经写入至：my1cam.yaml与my2cam.yaml中！"<<endl;
    cout << "快开始下一步工作吧~再见:)" << endl;
}


int main() {
    //////// 标定数据集的导入 //////////
    //右侧镜头
    vector<String> R_image_paths;//数据集中图像的路径
    string R_path = "./data_new/right";//图像名称格式
    glob(R_path,R_image_paths);// 将所有符合格式的图像路径存储到image_paths中
    //左侧镜头
    vector<String> L_image_paths;
    string L_path = "./data_new/left";
    glob(L_path, L_image_paths);

    ////// 世界坐标系三维坐标输入 //////
    vector<Point3f>objp; // 存储关键点在真实世界中的三维坐标
    for (int i = 0; i < CHESSBOARD[1]; i++)
    {
        for (int j = 0; j < CHESSBOARD[0]; j++)
            objp.push_back(Point3f(i, j, 0));//逐行标记内角点坐标
    }



    vector<Point2f> R_corners,L_corners;
    bool found1,found2;
    vector<vector<Point3f>> R_objPoints, L_objPoints;//为整个数据集创建其在真实世界的三维坐标集合。
    vector<vector<Point2f>> R_imgPoints,L_imgPoints;//图像上的关键点坐标位置
    ///////// 遍历数据集 && 确定关键点位置 ///////////
    std::cout<<"正在搜索标定点……"<<endl;
    for (int i = 0; i < R_image_paths.size(); i++)
    {
        R_src = imread(R_image_paths[i]);
        L_src = imread(L_image_paths[i]);
        cvtColor(R_src, R_img, COLOR_BGR2GRAY);
        cvtColor(L_src, L_img, COLOR_BGR2GRAY);
        //寻找关键点位置
        found1 = findChessboardCorners(R_img,
                                    Size(CHESSBOARD[0], CHESSBOARD[1]),R_corners,
                                    CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        found2 = findChessboardCorners(L_img,
            Size(CHESSBOARD[0], CHESSBOARD[1]), L_corners,
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        if (found1&&found2)//在图像上确定出内角点信息
        {
            TermCriteria criteria(2|1, 30, 0.001);//设置最大迭代次数与迭代终止条件
            //使用亚像素精准化关键点坐标位置
            cornerSubPix(R_img, R_corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cornerSubPix(L_img, L_corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            //在图像上显示检测到的关键点
            drawChessboardCorners(R_src, cv::Size(6, 9), R_corners, found1);
            drawChessboardCorners(L_src, cv::Size(6, 9), L_corners, found2);
            //存储关键点信息
            R_objPoints.push_back(objp);
            R_imgPoints.push_back(R_corners);
            L_objPoints.push_back(objp);
            L_imgPoints.push_back(L_corners);
        }
        //cv::imshow("Corner Detected Right", R_src);//展示图像
        //cv::imshow("Corner Detected Left", L_src);
        //waitKey(0);
    }
    cv::destroyAllWindows();
    cout << "标定点搜索完成！" << endl << endl << endl;

    ////////// 计算相机内外参数 ///////////
    cout << "开始计算镜头内外参，请稍等……" << endl;
    cv::calibrateCamera(R_objPoints, R_imgPoints, cv::Size(R_img.rows,R_img.cols), R_cameraMatrix, R_distCoeffs, R_R, R_T); // LEFT.K LEFT.D
    cv::calibrateCamera(L_objPoints, L_imgPoints, cv::Size(L_img.rows, L_img.cols), L_cameraMatrix, L_distCoeffs, L_R, L_T); // RIGHT.K RIGHT.D
    cout << "镜头内外参计算完成！" << endl;

    /*
    //左摄像头内参输入
    L_cameraMatrix = (Mat_<float>(3, 3) << 1363.068832071628, 0, 597.5310820482522, 0, 1364.502172300807, 380.6142772218984, 0, 0, 1);//相机内参矩阵
    L_distCoeffs = (Mat_<float>(1, 5) << -0.4401953292696111, 0.1678238673295687, 4.136661811836659e-05, 0.001406248426772503, 0.1224192370990156);//相机畸变向量
    //右摄像头内参输入
    R_cameraMatrix = (Mat_<float>(3, 3) << 1363.632211377158, 0, 646.0739633192405, 0, 1364.934222307617, 362.5550554222245, 0, 0, 1);//相机内参矩阵
    R_distCoeffs = (Mat_<float>(1, 5) << -0.4683699791765155, 0.5821333248038504, -2.920560964101862e-06, -0.001178312896501549, -1.229891391711476);//相机畸变向量
    */

    ///////////////////////// 立体视觉标定 ////////////////////////////

    Mat mapLx, mapLy, mapRx, mapRy;                         //映射表
    //Rect validROIL, validROIR;                              //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
    cv::Size imageSize(cv::Size(L_src.cols, L_src.rows));
    std::cout<<"开始立体视觉标定……"<<endl;
    double rms = stereoCalibrate(R_objPoints, L_imgPoints, R_imgPoints, L_cameraMatrix, L_distCoeffs, R_cameraMatrix, R_distCoeffs, imageSize, R, T, E, F, CALIB_USE_INTRINSIC_GUESS,
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
    /* 立体视觉矫正 stereoCalibrate()
     * 解释：该函数能进一步精准化相机内参与畸变矩阵，同时还能输出两个相机坐标系之间的转换关系矩阵 R T
     *  InputArrayOfArrays 	objectPoints, 标定点世界坐标
        InputArrayOfArrays 	imagePoints1, 标定点在相机1中坐标
        InputArrayOfArrays 	imagePoints2, 标定点在相机2中坐标
        InputOutputArray 	cameraMatrix1, 相机1 内参
        InputOutputArray 	distCoeffs1, 相机1畸变
        InputOutputArray 	cameraMatrix2, 相机2内参
        InputOutputArray 	distCoeffs2, 相机2畸变
        Size 	imageSize, 输入图像大小
        InputOutputArray 	R, 两个相机坐标系相互转换的旋转矩阵
        InputOutputArray 	T, 两个相机坐标系相互转换的位移矩阵
        OutputArray 	E, 本质矩阵(对极几何)
        OutputArray 	F, 基础矩阵
        OutputArray 	perViewErrors, 每张图像的重投影误差值
        int 	flags = CALIB_USE_INTRINSIC_GUESS , 初始值优化，由于我们输入了相机内参，我们使用我们计算好的结果作为迭代初始值——>CALIB_USE_INTRINSIC_GUESS
        TermCriteria 	criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6) 迭代设置
    )
    */
    cout << "立体视觉标定完成！RMS 误差为 = " << rms << endl << endl << endl;
    stereoRectify(L_cameraMatrix, L_distCoeffs, R_cameraMatrix, R_distCoeffs, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
    /*
    极线矫正函数 stereoRectify()
    cameraMatrix1-第一个摄像机的摄像机矩阵，即左相机相机内参矩阵，矩阵第三行格式应该为 0 0 1
    distCoeffs1-第一个摄像机的畸变向量
    cameraMatrix2-第一个摄像机的摄像机矩阵，即右相机相机内参矩阵，矩阵第三行格式应该为 0 0 1
    distCoeffs2-第二个摄像机的畸变向量
    imageSize-图像大小
    R- 相机之间的旋转矩阵，这里R的意义是：相机1通过变换R到达相机2的位姿
    T-  左相机到右相机的平移矩阵
    R1-输出矩阵，第一个摄像机的校正变换矩阵（旋转变换）极限矫正前相机坐标系下的3D坐标点——>极线校正后相机坐标系下的3D坐标点
    Output 3x3 rectification transform (rotation matrix) for the first camera. This matrix brings points given in the unrectified first camera's coordinate system to points in the rectified first camera's coordinate system. In more technical terms, it performs a change of basis from the unrectified first camera's coordinate system to the rectified first camera's coordinate system.
    R2-输出矩阵，第二个摄像机的校正变换矩阵（旋转矩阵）极限矫正前相机坐标系下的3D坐标点——>极线校正后相机坐标系下的3D坐标点
    Output 3x3 rectification transform (rotation matrix) for the second camera. This matrix brings points given in the unrectified second camera's coordinate system to points in the rectified second camera's coordinate system. In more technical terms, it performs a change of basis from the unrectified second camera's coordinate system to the rectified second camera's coordinate system.
    P1-输出矩阵，第一个摄像机在新坐标系下的投影矩阵 极线校正后相机坐标系下的3D坐标点——>极线矫正后图像平面上的2D像素坐标
    Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera, i.e. it projects points given in the rectified first camera coordinate system into the rectified first camera's image.
    P2-输出矩阵，第二个摄像机在新坐标系下的投影矩阵 极线校正后相机坐标系下的3D坐标点——>极线矫正后图像平面上的2D像素坐标
    Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera, i.e. it projects points given in the rectified first camera coordinate system into the rectified second camera's image.
    Q-4*4的深度差异映射矩阵
    flags-可选的标志有两种零或者 CV_CALIB_ZERO_DISPARITY ,如果设置 CV_CALIB_ZERO_DISPARITY 的话，该函数会让两幅校正后的图像的主点有相同的像素坐标。否则该函数会水平或垂直的移动图像，以使得其有用的范围最大
    alpha-拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
    newImageSize-校正后的图像分辨率，默认为原分辨率大小。
    validPixROI1-可选的输出参数，Rect型数据。其内部的所有像素都有效
    validPixROI2-可选的输出参数，Rect型数据。其内部的所有像素都有效
    */
    outputCameraParam();//参数写入
    Mat new_camera_matrix,frame,dst;
    frame = imread("./data_new/left/L_1.jpg");
    initUndistortRectifyMap(L_cameraMatrix, L_distCoeffs, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
    remap(frame, dst, mapLx, mapLy, INTER_LINEAR);
    imshow("left undistort",dst);
    imshow("handwrite",handwrite_Undistort(frame));
    imshow("stereo calibrate",show_stereoCalib());
    Mat srcR = imread("./data_new/right/R_1.jpg",COLOR_BGR2GRAY);
    //cv::Size imageSize(cv::Size(srcR.cols, srcR.rows));
    initUndistortRectifyMap(R_cameraMatrix, R_distCoeffs, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
    Mat rectifyImageR;
    remap(srcR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
    imshow("right undistort",rectifyImageR);
    waitKey(0);
    return 0;
}
