#pragma once
#include <string>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include <opencv.hpp>
#include <iostream>
#include<iomanip>
#include ".\TinyXML\tinyxml.h"
#include ".\TinyXML\tinystr.h"
// for console window
#include <conio.h> 

using namespace std;
//using namespace cv;

using cv::Mat;
using cv::Mat_;
using cv::Vec4f;
using cv::Point3f;
using cv::Vec3f;
using cv::Vec4i;
using cv::Vec6f;
using cv::Point2i;
using cv::Point2f;
using cv::Point2d;
using cv::Point3d;
using cv::Size;
using cv::Point;
using cv::Rect;
using cv::Scalar;
using cv::RotatedRect;
using cv::String;
using cv::SVD;
//28.152,12,7
//26.928 11 8
//37.2 11 5
#define SQUARE_SIZE 25.048			//<棋盘格方格尺�?
#define WIDTH_CORNER_COUNT 17		//<长边角点个数
#define HEIGHT_CORNER_COUNT 8		//<短边角点个数
#define CAMERA_NUM 2				//<操作的相机数�?
////chengwei added
#define HEADRADIUS 0.99999f;
//光笔测头半径，用作半径补�?

typedef Vec3f TagPoint2f;
typedef Vec4f TagPoint3f;
typedef Point3f SurfaceNormal;

//#define BLUR
//#define CANNY
//#define THRESHOL

//opencv3.0
#define TRUE 1
#define FALSE 0

enum PointTag
{
    TAG1,
    TAG2,
    TAG3,
    TAG4,
    TAG5,
    TAG6,
    TAG7
};

enum SystemMode
{
    SINGLE_CAM,
    DOUBLE_CAM,
    TRIPLE_CAM
};

enum CamPosition
{
    LEFT,
    MID,
    RIGHT
};

struct RT
{
    cv::Mat R; //<3X1
    cv::Mat T; //<3X1
};


class CalibrationData
{
public:
    CalibrationData() {}

    void ErasePoint(int frameNum, int pointNum)//删掉frameNum中的第pointNum个点�?为第一个点�?
    {
        vector< vector<Point3f> >::iterator IterPoint3fs;
        vector< vector<Point2f> >::iterator IterPoint2fs;
        vector<Point3f>::iterator IterPoint3f;
        vector<Point2f>::iterator IterPoint2f;
        int i = 0;
        for(vector<int>::iterator IterFrame = frameNumList.begin(); IterFrame != frameNumList.end(); i++)
        {
            if (*IterFrame == frameNum)
            {
                IterPoint3fs = plane3dPntsVec.begin() + i;
                IterPoint2fs = plane2dPntsVec.begin() + i;
                if (i >= plane3dPntsVec.size() || i >= plane2dPntsVec.size())
                    return;
                IterPoint3f = IterPoint3fs->begin();
                IterPoint2f = IterPoint2fs->begin();
                if (pointNum >= plane3dPntsVec[i].size() || pointNum >= plane2dPntsVec[i].size())
                    return;
                IterPoint3fs->erase(IterPoint3f + pointNum);
                IterPoint2fs->erase(IterPoint2f + pointNum);
                break;
            }
            else
                IterFrame++;
        }
    }


    int imgHeight;								//<图像的高
    int imgWidth;								//<图像的宽
    vector<int> frameNumList;					//<图像的索�?
    //std::vector<int> CornerNumPerFrame;       //<每一帧图像角点个�?
    vector< vector<Point3f> > plane3dPntsVec;		//<标定板三维平面坐�?
    vector< vector<Point2f> > plane2dPntsVec;		//<标定板二维坐�?

};


class CamPara
{
public:
    CamPara()
    {
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                CameraIntrinsic[i][j]=0;
        CameraIntrinsic[2][2] = 1;
        for(int i=0;i<4;i++) {DistortionCoeffs[i]=0;kcError[i]=0;}
        ReprojectionError[0]=0;ReprojectionError[1]=0;
        fcError[0]=0;fcError[1]=0;
        ccError[0]=0;ccError[1]=0;
    }
    double CameraIntrinsic[3][3];	//<相机内参�?
    double DistortionCoeffs[4];		//<相机畸变参数
    std::vector<RT> imgRTVec;		//<标定板外参数
    Mat parallelCamR;					//<平行相机视图的R
    double fcError[2];
    double ccError[2];
    double kcError[4];
    double ReprojectionError[2];		//<重投影误差标准差
    std::vector<double> reprojectNormErr;
    double totalReproNormErr;
};

///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// 程伟 ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

enum
{
    PNP_ITERATIVE = 0,
    PNP_EPNP = 1, // F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    PNP_P3P = 2, // X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
    //以下两个需要opencv3.0以上版本才支�?
    PNP_DLS = 3, //Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. “A Direct Least-Squares (DLS) Method for PnP�?
    PNP_UPNP = 4 //Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer.
    //“Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation�?
    //In this case the function also estimates the parameters f_x and f_y assuming that both have the same value.
    //Then the cameraMatrix is updated with the estimated focal length.
};

enum CAMBEHAVIOR
{
    CAMCALIBRATON,
    LIGHTPENCALIBRATION,
    MESSURE
};

enum MESMODEL
{
    MESSINGLEPOINT,
    MESPLANEFIRST,
    MESPLANESECOND,
    MESCIRCLECYLINDER,
    MESPOINTTOPOINT
};

struct ImageSaveDir
{
    string calibrateCamDir;
    string calibrateProbeDir;
    string probePntDir;
};

class Mat34
{
public:
    Mat34()
    {
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<4;j++)
            {
                mat[i][j] = 0;
            }
        }
    }
    double mat[3][4];
};

class CamGroupPara
{
public:
    CamGroupPara()
    {
        for(int i=0;i<3;i++)
        {
            left2MidRotVector[i] = 0;
            left2MidTraVector[i] = 0;
            right2MidRotVector[i] = 0;
            right2MidTraVector[i] = 0;
            right2LeftRotVector[i] = 0;
            right2LeftTraVector[i] = 0;
        }
    }

public:
    double left2MidRotVector[3];
    double left2MidTraVector[3];
    double right2MidRotVector[3];
    double right2MidTraVector[3];
    double right2LeftRotVector[3];
    double right2LeftTraVector[3];
};

//包括每次测量的侧头中心坐标（测量坐标系下的）和测量表面的法线
struct CenterFeartures
{
    Point3f center1;
    Point3f center2;
    Point3f center3;
    SurfaceNormal feature;//测量表面的法�?
};

//经过半径补偿的被测表面三个点的坐标（在测量坐标系下的）以及测量表面的法线
struct MessureResult
{
    Point3f point1;
    Point3f point2;
    Point3f point3;
    SurfaceNormal feature;//测量表面的法�?
};

//光笔标定参数
struct LightPenPara
{
    vector<TagPoint3f> FeatursPoints;
    Point3f CenterPoint;
};

//平面模型
class SPlane
{
public:
    SurfaceNormal normal;
    Point3f orignal;
};

//柱面模型
class CircleCylinder
{
public:
    SurfaceNormal axisNormal;//单位向量
    Point3f orignal;//位于轴线上的一�?
    float r;//圆柱半径
};

class Line
{
public:
    SurfaceNormal normal;//方向向量
    Point3f orignal;//直线上一�?

};

///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// 郑泽�?//////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

#define MultiNum  5		//大小圆长轴放大倍数
#define ALLOWERROR 0.09		//方向允许误差
#define ThreshLevel 250	//圆提取阈�?
#define AreaMax 10000		//轮廓筛选面积最大�?
#define AreaMin 100			//轮廓筛选面积最小�?
#define RATE    4			//大小圆长轴比�?
#define LRATE   4         //椭圆长短轴比，进行标靶标定时LRATE应选大值推�?，进行正常的识别时选小值，推荐2~2.2

//标靶标定相关
#define FIR_TO_TW 101
#define FIR_TO_TH 102
#define FIR_TO_FO 103
#define FIR_TO_FI 104

enum FeaCircleFlag   //特征圆标�?
{
    CIR_0=0,   //零表示还为做标识
    CIR_1=1,
    CIR_2,
    CIR_3,
    CIR_4,
    CIR_5,
    CIR_6,
    CIR_7,
    CIR_a,
    CIR_b,
    CIR_c,
};
//enum AreaTag
//{
//	AREA_1=1,
//	AREA_2,
//	AREA_3,
//};
enum FaceFlag  //面编号标�?
{
    Face_1=1,   //零表示还未做标识
    Face_2,
    Face_3,
    Face_4,
    Face_5,
    Face_6,
};

struct FeaCircle  //存放特征圆信息的结构�?
{
    Point2f center;
    float height,width;
    enum FeaCircleFlag flag;
};

struct SH_Ellipse//存放提取椭圆信息结构�?
{
    Point2f center;
    float macroaxis,brachyaxis;
};

struct circle_inf		//定义圆特征信息结构体
{
    Point2f center;
    float height;
    float width;
    vector<float> dis2othercir;			//到其他圆的距�?
    enum FeaCircleFlag flag;			//圆号码标记；
    int markflag;						//用于识别一号圆的标�?
    vector<int> cirflag;				//用于存放于该圆距离小�?.25D的圆的编�?
    double dist2large1[2];				//到一号大圆的距离,一个存储X方向一个储存Y方向
    int dist2large1Flag;				//用来存储各大圆到一号大圆距离的大小顺序标志，用于找出距离一号大圆最近的两个大圆
};

struct mark_area				 //存储标记面的结构�?
{
    //重载等号操作
    mark_area& operator=(mark_area& value)
    {
        large_circle = value.large_circle;
        small_circle = value.small_circle;
        Face=value.Face;
        return *this;
    }
    vector <Point2f> large_circle;
    vector <Point2f> small_circle;
    enum FaceFlag Face;
};

struct StereoEllipse
{
    CvPoint3D32f  NormalVector;
    CvPoint3D32f  center;
    double r;
};

class ObjectPara
{
public:
    ObjectPara()
    {
        vector<Point3f> FacePnt;
        for (int i=0;i<7;i++)
        {
            FacePnt.push_back(Point3f(0,0,0));
        }
        for (int i=0;i<5;i++)
        {
            FacePnts[i] = FacePnt;
        }
        for (int i=0;i<4;i++)
        {
            RT2one[i]=cv::Mat::zeros(4,4,CV_64FC1);
        }
    }
    vector<Point3f> FacePnts[5];
    Mat RT2one[4];
};

typedef vector <RotatedRect> EllipseBox;
typedef vector <mark_area> MarkArea;


//位姿结构体
