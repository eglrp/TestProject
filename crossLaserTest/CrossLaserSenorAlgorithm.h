//十字激光传感器的内置算法，与硬件无关
#pragma once
#include <vector>
#include <cv.h>
#include "Algorithm.h"
using namespace std;

	//十字激光传感器算法实现
class CrossLaserSenorAlgorithm
	{
		typedef vector<cv::Point2f> points_2d;
		friend class CrossLaserSensor;
		friend class ScannerCalibrate;
	public:
		CrossLaserSenorAlgorithm();
		~CrossLaserSenorAlgorithm();
		//导入传感器内参
		bool readIntrinsicPara(const string &fileName);
		//标定十字激光传感器的内参即：标定十字激光两个刀光平面的位姿
		//note:激光条纹中心不需要预先进行校正，函数体内会校正。
		bool estimateIntrinsicPara(const CamPara &camPara, const vector<RT> &targetPlaneRT1, const vector<RT> &targetPlaneRT2, 
			const vector<points_2d> &stripes1, const vector<points_2d> &stripes2);
		//标定十字激光传感器的内参即：标定十字激光两个刀光平面的位姿
		bool estimateIntrinsicPara(const CamPara &camPara, double *planePara1, double *planePara2);
		//由激光图像解析激光条纹中心的三维坐标系（传感器坐标系下）
		bool remodel(const cv::Mat &src, vector<cv::Point3f> &pointCloud, const cv::Rect *ROI = NULL);
		
	protected:
		CamPara m_camPara; //相机参数矩阵
		cv::Mat m_transMat1, m_transMat2; //从图像坐标系到两个光刀平面的坐标的转化矩阵
		string m_errInf; //错误信息
	private:
		bool isCalibrated; //是否已经标定标识符
		cv::Mat map1, map2; //图像畸变矫正映射矩阵
		cv::Mat gausk; //高斯滤波核
		cv::Mat rt1, rt2;//两刀光平面坐标系在传感器坐标系下的位姿矩阵 //todo::和m_transMat1, m_transMat2可以合并成一个矩阵
	};


