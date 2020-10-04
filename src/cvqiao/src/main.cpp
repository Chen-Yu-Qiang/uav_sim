
/*#include <iostream>
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <ros/ros.h>
using namespace std;
void callbackfun(){

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 100, callbackfun);
    ros::spin();
    return 0;
}
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

//#include <io.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <sstream>
#include <time.h>

using namespace cv;
using namespace std;
static const std::string OPENCV_WINDOW = "Image window";
Size board_size = Size(6, 10);   //內角點數量，垂直*水平
Size square_size = Size(20, 20); //棋盤格方塊大小mm*mm

Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); //鏡頭內部參數
Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));   // 鏡頭的5個畸變係數k1k2p1p2k3
ofstream fout("caliberation_result.txt");
bool find_outmat_fromAPicture(Mat , Size , Size , Mat &, Mat &, Mat &, Mat &);
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int i=0;
    int ok=0;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/tello/image_raw", 10,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 10);

        //cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        // Update GUI Window
        //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //cv::waitKey(3);

        // Output modified video stream
        
        /*Mat aaa = cv_ptr->image;
        ROS_INFO("1");
        if(i%30==100){
            imshow("a",aaa);
            waitKey(1);
            
        }*/
        image_pub_.publish(cv_ptr->toImageMsg());
        /*Mat rvecsMat;
        Mat tvecsMat;
        i++;
            
        if(find_outmat_fromAPicture(aaa, board_size, square_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat)){
            
        }
        ok++;
        cout<<ok<<endl;
        
        //cout<<tvecsMat;
        //cout<<endl;
        Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
        
        try
        {
            Rodrigues(rvecsMat, rotation_matrix);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        
        fout << "r(:,:," << i + 1 << ")=";
        fout << rotation_matrix << ";" << endl;
        fout << "t(:," << i + 1 << ")=";
        fout << tvecsMat << ";" << endl;*/
    
    }
};
void getFilesName(string &File_Directory, vector<string> &FilesName)
{
    string fullFilePath;
    fullFilePath = File_Directory + "/60.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/80.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/100.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/120.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/140.png";
    FilesName.push_back(fullFilePath);
    return ;
    fullFilePath = File_Directory + "/160.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/180.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/200.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/220.png";
    FilesName.push_back(fullFilePath);

    fullFilePath = File_Directory + "/60 R90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/80 R90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/100 R90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/120 R90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/140 R90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/160 R90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/180 R90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/200 R90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/220 R90.png";
    FilesName.push_back(fullFilePath);

    fullFilePath = File_Directory + "/60 L90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/80 L90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/100 L90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/120 L90.png";
    FilesName.push_back(fullFilePath);
    fullFilePath = File_Directory + "/140 L90.png";
    FilesName.push_back(fullFilePath);
}
void find_point(vector<string> &FilesName, Size board_size, vector<vector<Point2f> > &image_points_seq)
{

    cout << "开始提取角点………………" << FilesName.size() << endl;
    int image_count = 0; // 图像数量
    Size image_size;     // 图像的尺寸
    ofstream fout2("ChessboardCorners_result.txt");
    vector<Point2f> image_points; // 缓存每幅图像上检测到的角点

    for (int i = 0; i < FilesName.size(); i++)
    {
        image_count++;

        // 用于观察检验输出
        cout << "image_count = " << image_count << endl;
        Mat imageInput = imread(FilesName[i]);
        if (imageInput.empty())
        {
            //检查是否读取图像
            cout << "Error! Input image cannot be read...\n";
        }
        //imshow("Camera Calibration", imageInput);
        //waitKey(0);

        /* 提取角点 */
        bool ok = findChessboardCorners(imageInput, board_size, image_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
        if (0 == ok)
        {
            //cout << "第" << image_count << "张照片提取角点失败，请删除后，重新标定！" << endl; //找不到角点
            //imshow("失败照片", imageInput);
            //waitKey(0);
        }
        else
        {
            Mat view_gray;
            cout << "imageInput.channels()=" << imageInput.channels() << endl;
            cvtColor(imageInput, view_gray, CV_RGB2GRAY);

            /* 亚像素精确化 */
            //find4QuadCornerSubpix(view_gray, image_points, Size(5, 5)); //对粗提取的角点进行精确化
            cv::cornerSubPix(view_gray, image_points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 20, 0.01));

            image_points_seq.push_back(image_points); //保存亚像素角点
            fout2 << "%第" << image_count << "张照片" << endl;

            for (int j = 0; j < image_points.size(); j++)
            {
                fout2 << "p(:," << j + 1 << "," << image_count << ")=[" << image_points[j].x << "," << image_points[j].y << "];" << endl;
            }
            /* 在图像上显示角点位置 */
            //drawChessboardCorners(view_gray, board_size, image_points, true);

            //imshow("Camera Calibration", view_gray); //显示图片
            //waitKey(0);                              //暂停0.1S
        }
    }
    //cout << "角点提取完成！！！" << endl;
}
void find_mat(vector<vector<Point2f> > &image_points_seq, Size board_size, Size square_size, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecsMat, vector<Mat> &tvecsMat)
{
    ofstream fout("caliberation_result.txt");  // 保存标定结果的文件
    int image_count = image_points_seq.size(); // 图像数量
    Size image_size(960, 720);                 // 图像的尺寸
    cout << "image_size.width = " << image_size.width << endl;
    cout << "image_size.height = " << image_size.height << endl;
    /*棋盘三维信息*/
    vector<vector<Point3f> > object_points_seq; // 保存标定板上角点的三维坐标

    for (int t = 0; t < image_count; t++)
    {
        vector<Point3f> object_points;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = i * square_size.width;
                realPoint.y = j * square_size.height;
                realPoint.z = 0;
                object_points.push_back(realPoint);
            }
        }
        object_points_seq.push_back(object_points);
    }

    /* 运行标定函数 */
    double err_first = calibrateCamera(object_points_seq, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, CV_CALIB_FIX_K3);
    fout << "重投影误差1：" << err_first << "像素" << endl
         << endl;
    cout << "标定完成！！！" << endl;

    cout << "开始评价标定结果………………";
    double total_err = 0.0; // 所有图像的平均误差的总和
    double err = 0.0;       // 每幅图像的平均误差
    double totalErr = 0.0;
    double totalPoints = 0.0;
    vector<Point2f> image_points_pro; // 保存重新计算得到的投影点

    for (int i = 0; i < image_count; i++)
    {

        projectPoints(object_points_seq[i], rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points_pro); //通过得到的摄像机内外参数，对角点的空间三维坐标进行重新投影计算

        err = norm(Mat(image_points_seq[i]), Mat(image_points_pro), NORM_L2);

        totalErr += err * err;
        totalPoints += object_points_seq[i].size();

        err /= object_points_seq[i].size();
        //fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
        total_err += err;
    }
    fout << "重投影误差2：" << sqrt(totalErr / totalPoints) << "像素" << endl
         << endl;
    fout << "重投影误差3：" << total_err / image_count << "像素" << endl
         << endl;

    //保存定标结果
    cout << "开始保存定标结果………………" << endl;
    Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    fout << "相机内参数矩阵：" << endl;
    fout << cameraMatrix << endl
         << endl;
    fout << "畸变系数：\n";
    fout << distCoeffs << endl
         << endl
         << endl;
    for (int i = 0; i < image_count; i++)
    {
        //fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
        //fout << rvecsMat[i] << endl;

        /* 将旋转向量转换为相对应的旋转矩阵 */
        Rodrigues(rvecsMat[i], rotation_matrix);
        /*fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
        fout << rotation_matrix << endl;
        fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
        fout << tvecsMat[i] << endl
             << endl;*/
        fout << "r(:,:," << i + 1 << ")=";
        fout << rotation_matrix << ";" << endl;
        fout << "t(:," << i + 1 << ")=";
        fout << tvecsMat[i] << ";" << endl;
    }
    cout << "定标结果完成保存！！！" << endl;
    fout << endl;
}
void m_calibration(vector<string> &FilesName, Size board_size, Size square_size, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecsMat, vector<Mat> &tvecsMat)
{
    ofstream fout("caliberation_result.txt"); // 保存标定结果的文件

    cout << "开始提取角点………………" << FilesName.size() << endl;
    int image_count = 0; // 图像数量
    Size image_size;     // 图像的尺寸
    ofstream fout2("ChessboardCorners_result.txt");
    vector<Point2f> image_points;             // 缓存每幅图像上检测到的角点
    vector<vector<Point2f> > image_points_seq; // 保存检测到的所有角点

    for (int i = 0; i < FilesName.size(); i++)
    {
        image_count++;

        // 用于观察检验输出
        cout << "image_count = " << image_count << endl;
        Mat imageInput = imread(FilesName[i]);
        if (imageInput.empty())
        {
            //检查是否读取图像
            cout << "Error! Input image cannot be read...\n";
        }
        //imshow("Camera Calibration", imageInput);
        //waitKey(0);
        if (image_count) //读入第一张图片时获取图像宽高信息
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            cout << "image_size.width = " << image_size.width << endl;
            cout << "image_size.height = " << image_size.height << endl;
        }

        /* 提取角点 */
        bool ok = findChessboardCorners(imageInput, board_size, image_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
        if (0 == ok)
        {
            cout << "第" << image_count << "张照片提取角点失败，请删除后，重新标定！" << endl; //找不到角点
            //imshow("失败照片", imageInput);
            //waitKey(0);
        }
        else
        {
            Mat view_gray;
            cout << "imageInput.channels()=" << imageInput.channels() << endl;
            cvtColor(imageInput, view_gray, CV_RGB2GRAY);

            /* 亚像素精确化 */
            //find4QuadCornerSubpix(view_gray, image_points, Size(5, 5)); //对粗提取的角点进行精确化
            cv::cornerSubPix(view_gray, image_points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 20, 0.01));

            image_points_seq.push_back(image_points); //保存亚像素角点
            fout2 << "%第" << image_count << "张照片" << endl;

            for (int j = 0; j < image_points.size(); j++)
            {
                fout2 << "p(:," << j + 1 << "," << image_count << ")=[" << image_points[j].x << "," << image_points[j].y << "];" << endl;
            }
            /* 在图像上显示角点位置 */
            drawChessboardCorners(view_gray, board_size, image_points, true);

            //imshow("Camera Calibration", view_gray); //显示图片
            //waitKey(0);                              //暂停0.1S
        }
    }
    cout << "角点提取完成！！！" << endl;

    /*棋盘三维信息*/
    vector<vector<Point3f> > object_points_seq; // 保存标定板上角点的三维坐标

    for (int t = 0; t < image_count; t++)
    {
        vector<Point3f> object_points;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = i * square_size.width;
                realPoint.y = j * square_size.height;
                realPoint.z = 0;
                object_points.push_back(realPoint);
            }
        }
        object_points_seq.push_back(object_points);
    }

    /* 运行标定函数 */
    double err_first = calibrateCamera(object_points_seq, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, CV_CALIB_FIX_K3);
    fout << "重投影误差1：" << err_first << "像素" << endl
         << endl;
    cout << "标定完成！！！" << endl;

    cout << "开始评价标定结果………………";
    double total_err = 0.0; // 所有图像的平均误差的总和
    double err = 0.0;       // 每幅图像的平均误差
    double totalErr = 0.0;
    double totalPoints = 0.0;
    vector<Point2f> image_points_pro; // 保存重新计算得到的投影点

    for (int i = 0; i < image_count; i++)
    {

        projectPoints(object_points_seq[i], rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points_pro); //通过得到的摄像机内外参数，对角点的空间三维坐标进行重新投影计算

        err = norm(Mat(image_points_seq[i]), Mat(image_points_pro), NORM_L2);

        totalErr += err * err;
        totalPoints += object_points_seq[i].size();

        err /= object_points_seq[i].size();
        //fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
        total_err += err;
    }
    fout << "重投影误差2：" << sqrt(totalErr / totalPoints) << "像素" << endl
         << endl;
    fout << "重投影误差3：" << total_err / image_count << "像素" << endl
         << endl;

    //保存定标结果
    cout << "开始保存定标结果………………" << endl;
    Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    fout << "相机内参数矩阵：" << endl;
    fout << cameraMatrix << endl
         << endl;
    fout << "畸变系数：\n";
    fout << distCoeffs << endl
         << endl
         << endl;
    for (int i = 0; i < image_count; i++)
    {
        //fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
        //fout << rvecsMat[i] << endl;

        /* 将旋转向量转换为相对应的旋转矩阵 */
        Rodrigues(rvecsMat[i], rotation_matrix);
        /*fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
        fout << rotation_matrix << endl;
        fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
        fout << tvecsMat[i] << endl
             << endl;*/
        fout << "r(:,:," << i + 1 << ")=";
        fout << rotation_matrix << ";" << endl;
        fout << "t(:," << i + 1 << ")=";
        fout << tvecsMat[i] << ";" << endl;
    }
    cout << "定标结果完成保存！！！" << endl;
    fout << endl;
}

void m_undistort(vector<string> &FilesName, Size image_size, Mat &cameraMatrix, Mat &distCoeffs)
{

    Mat mapx = Mat(image_size, CV_32FC1); //X 坐标重映射参数
    Mat mapy = Mat(image_size, CV_32FC1); //Y 坐标重映射参数

    Mat R = Mat::eye(3, 3, CV_32F);
    cout << "保存矫正图像" << endl;
    string imageFileName; //校正后图像的保存路径
    stringstream StrStm;
    string temp;

    for (int i = 0; i < FilesName.size(); i++)
    {
        Mat imageSource = imread(FilesName[i]);

        Mat newimage = imageSource.clone();

        //方法一：使用initUndistortRectifyMap和remap两个函数配合实现
        //initUndistortRectifyMap(cameraMatrix,distCoeffs,R, Mat(),image_size,CV_32FC1,mapx,mapy);
        //  remap(imageSource,newimage,mapx, mapy, INTER_LINEAR);

        //方法二：不需要转换矩阵的方式，使用undistort函数实现
        undistort(imageSource, newimage, cameraMatrix, distCoeffs);

        StrStm << i + 1;
        StrStm >> temp;
        imageFileName = "矫正后图像//" + temp + "_d.jpg";
        imwrite(imageFileName, newimage);

        StrStm.clear();
        imageFileName.clear();
    }
    std::cout << "保存结束" << endl;
}

void find_point_fromCSV(string &CSV_name, vector<vector<Point2f> > &image_points_seq)
{
    ifstream in_file(CSV_name, ios::in);
    string a_line;
    vector<Point2f> image_points;
    while (getline(in_file, a_line))
    {
        if (a_line == "=====")
        {

            cout << "total " << image_points.size() << " point in a picture";
            image_points_seq.push_back(image_points);
            cout << " ,total " << image_points_seq.size() << " picture";
            cout << endl
                 << "===" << endl;
            image_points.clear();
        }
        else
        {
            Point2f a_point;
            stringstream ss(a_line);
            string a_val;
            getline(ss, a_val, ',');
            a_point.x = stof(a_val);
            getline(ss, a_val, ',');
            a_point.y = stof(a_val);
            cout << a_point << endl;
            image_points.push_back(a_point);
        }
    }
}
bool find_outmat_fromAPicture(Mat imageInput, Size board_size, Size square_size, Mat &cameraMatrix, Mat &distCoeffs, Mat &rvecsMat, Mat &tvecsMat)
{
    //cout << "开始提取角点………………" << endl;
    Size image_size;              // 图像的尺寸
    vector<Point2f> image_points; // 缓存每幅图像上检测到的角点

    // 用于观察检验输出
    if (imageInput.empty())
    {
        //检查是否读取图像
        cout << "Error! Input image cannot be read...\n";
    }
    //imshow("Camera Calibration", imageInput);
    //waitKey(0);

    /* 提取角点 */
    bool ok = findChessboardCorners(imageInput, board_size, image_points, CALIB_CB_FAST_CHECK);
    //bool ok = findChessboardCorners(imageInput, board_size, image_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
    if (0 == ok)
    {
        cout << "no find point" << endl; //找不到角点
        return 0;
    }
    else
    {
        findChessboardCorners(imageInput, board_size, image_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
        Mat view_gray;
        //cout << "imageInput.channels()=" << imageInput.channels() << endl;
        cvtColor(imageInput, view_gray, CV_RGB2GRAY);

        /* 亚像素精确化 */
        //find4QuadCornerSubpix(view_gray, image_points, Size(11, 11)); //对粗提取的角点进行精确化
        cv::cornerSubPix(view_gray, image_points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 20, 0.01));

        /* 在图像上显示角点位置 */
        drawChessboardCorners(view_gray, board_size, image_points, true);

        //imshow("Camera Calibration", view_gray); //显示图片
        //waitKey(1);                              //暂停0.1S
    }

    //cout << "角点提取完成！！！" << endl;
    vector<Point3f> object_points;
    for (int i = 0; i < board_size.height; i++)
    {
        for (int j = 0; j < board_size.width; j++)
        {
            Point3f realPoint;
            /* 假设标定板放在世界坐标系中z=0的平面上 */
            realPoint.x = i * square_size.width;
            realPoint.y = j * square_size.height;
            realPoint.z = 0;
            object_points.push_back(realPoint);
        }
    }
    return solvePnP(object_points, image_points, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);

}
int main(int argc, char **argv)
{
    string File_Directory1 = "/home/yuqiang/catkin_ws/src/cvqiao/src";
    vector<string> FilesName1;
    //getFilesName(File_Directory1, FilesName1);
    vector<Mat> rvecsMat;
    vector<Mat> tvecsMat;
    vector<vector<Point2f> > image_points_seq;
    //find_point(FilesName1, board_size, image_points_seq);
    //string CSV_name = "/home/yuqiang/catkin_ws/src/roscppcvcalibratecamera/src/pointMATLAB.csv";
    //find_point_fromCSV(CSV_name, image_points_seq);
    //find_mat(image_points_seq, board_size, square_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
