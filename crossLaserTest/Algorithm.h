#pragma once
#include "SharedHead.h"
#include "cvtool.h"
#include <time.h>
#include <queue>
#define  PI 3.141592654
class CAlgorithm
{
public:
	//激光条纹中心，index表示遍历方向的行/列索引，pt表示提取到的条纹中心的在当前行/列的亚像素位置。
	struct laserCenter
	{
		int index;
		vector <float> pt;
	};
	typedef vector< vector<Point2f> > clusterPnts; //平面点集

public:
	//TODO:待编写的几个算法
	//由球上的经纬线求已经半径球的球心位置
	static bool getBallCenter(const vector<Point3f>srcPnts, const double ridus, Point3f &BallCenter);
	//编写新的空间平面拟合算法
	static bool FitPlane2(vector<Point3f>srcPnts, double *PlaneParam); //实验中发现现有的平面拟合算法和geometric的拟合结果有些出入

	//计算光条图像坐标点和光条平面坐标系下的物理坐标点的单应矩阵
	//note: Hmat矩阵的数值通常很小，直接保存，重新读取时数值精度不够，建议求逆后再保存。
	static bool CaculateHMatric(const double *planeParams, const CamPara &camParam, cv::Mat &camFrameInLaserFrame, cv::Mat &Hmat);
	//end todo
	//用求极线的方法计算
	//空间圆拟合
	static bool FitCircle3D(vector<Point3f>srcPnts, circle3d &circle3dParam);
	//空间平面最小二乘拟合
	static bool FitPlane(vector<Point3f>srcPnts, double *PlaneParam);
	//反向投影：将世界坐标系下的三维点投影到图像坐标系中，并得出投影误差
	static double projectPointToImg(const vector<Point3f> &srcPnts, const vector<Point2f> &srcImgPnts,
		const Mat& intrinsicPara, const Mat& distCoeffs, const Mat &r, const Mat &t, vector<Point2f> &dstPnt);
	//获得行最大灰度像素
	static bool getRowMaxLocMid(Mat& rowMat, int colStart, int colEnd, int& maxValue, int& colMaxMid, const bool& MaxMidType);
	//获得行中最大灰度像素,输入为行指针
	static bool getRowMaxLocMid(uchar* rowPtr, int colStart, int colEnd, int& maxValue, int& colMaxMid, const bool& MaxMidType);
	//获得行中前几个最大灰度像素,输入为行指针
	static bool getMultMax(uchar* rowPtr, int colStart, int colEnd, int windowwidth, int findNum, vector <uchar>& maxValue, vector <int>& colMax);
	//获取列最大值，同获取行最大值的操作方式有所不同，行是连续的地址，列不是
	static bool getColMaxLocMid(Mat& colMat, int rowStart, int rowEnd, int& maxValue, int& rowMaxMid, const bool& MaxMidType);
	//高斯核
	static bool GaussKNew(Mat& gaussK, int Krow, int Kcol, float Ksig);
	//部分高斯滤波
	static bool PartGaussianBlur(Mat& srcImg, Mat& srcCpy, int rowCurrent, int colStart, int colEnd, Mat& gaussK);
	//列图像部分高斯滤波
	static bool PartGaussianBlur_col(Mat& srcImg, Mat& srcCpy, int curCol, int rowStart, int rowEnd, Mat& gaussK);
	//利用正向投影（将图像点投影到世界坐标系）评价pnp位s姿估计误差
	//激光条纹中心提取算法——行方向（指定roi版本）
	static bool LaserCenterDetect_row(Mat& src, const int Threshold, const int windowwidth, int centerRange,
		vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI);
	//激光条纹中心提取算法——行方向（无roi版本）
	static  bool LaserCenterDetect_row(Mat& src, const int Threshold, const int windowwidth, int centerRange,
		vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType);
	//激光条纹中心提取算法（无roi版本）
	static bool LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange,
		vector<Point2f> &LaserCenter,double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType);
	////激光条纹提取（列遍历）
	static bool LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange,
		vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI);

	//确定条纹宽度、前景、背景亮度
	static bool getWTT(uchar *src, cv::Point pos, double &I1, double &I2, double &W);

	//对点云进行刚体变化
	static bool rigidTransformPointCould(const Mat& rigidTransformRT, vector<Point3f> &srcPnts, vector<Point3f> &dstPnts);
	//通过三点在不同坐标系下坐标系，解算坐标系之间的刚体变换
	static void rigidTransformFrom3Pnt(const vector<Point3f>pntsInRef, const vector<Point3f> pntsInLocal, Mat &rt);
	//由三点构造局部坐标系,中间点作为坐标系原点
	static void frameFrom3Pnts(const vector<Point3f>pnts, Mat &rt);
	//坐标系绕Z轴旋转ang角的旋转变换矩阵
	static Mat rotz(double rotAngle,bool radFlag = false);
#if 0
	//光条中心提取 
	bool LaserCenterDetector(const Mat& srcimage, const cv::Rect mask, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter);
	//光条中心提取 无roi
	bool LaserCenterDetector(const Mat& srcimage, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter);
#endif

};

