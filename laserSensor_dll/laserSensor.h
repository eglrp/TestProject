#pragma once
#include <cv.h>
#include<string>

//boost serialization
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

using namespace std;


static int deviceZatRef = 0;
static float pulseDis = 1; //设备Z轴一个脉冲对应的物理尺寸，单位：mm
//获取设备Z轴的坐标
int getZInDevice(double Z);

class laserSensor
{
public:
	struct LaserExtractPara{
		LaserExtractPara();
		int threasold; //背景和前景对比度阈值
		int windowWidth; //条纹宽度预估值
		cv::Mat guassK; //高斯核

		//序列化和反序列化结构体
		template<class Archive>
		void serializeA(Archive& ar, const unsigned int version){
			ar & threasold;
			ar & windowWidth;
			cvtool::cvmatSerializeA(ar, guassK, version);
		}
	};
	//设备标定参数
	struct SensorPara{
		SensorPara();
		double ratio; //平行激光条纹图像距离与实际高度变化的比例因子
		double refLaserLine[3]; //参考平面的激光条纹直线参数 ax + by + c = 0中的a b c,注意：a,b代表的是与直线垂直的向量。
		double hpara[3]; //单应矩阵结算参数：第一个为Z轴移动量和像素变化量的比例因子；2，3为xy变化方向
		cv::Mat refHmat; //参考位置处的单应性矩阵
		cv::Mat worldInHand; //世界坐标的xy平面在TCP坐标系的xy平面的变换矩阵（只适用于世界坐标的Z轴和设备Z轴平行的情况）
		LaserExtractPara laserExtractPara;//激光条纹提取参数
		//test 
		double polypara[3];

		//序列化和反序列化结构体
		template<class Archive>
		void serializeA(Archive& ar, const unsigned int version){
			ar & ratio;
			ar & refLaserLine;
			ar & hpara;
			cvtool::cvmatSerializeA(ar,refHmat, version);
			cvtool::cvmatSerializeA(ar,worldInHand, version);
			laserExtractPara.serializeA(ar, version);
		}
	};
public:
	laserSensor();
	~laserSensor();

	//获取条纹指示的高度(相对参考位置)
	double getZ(cv::Mat &src, const cv::Rect &ROI = cv::Rect(0,0,0,0));
	//由图像点获取该点在设备xy坐标系下的位置
	//para pointInImg input 在搜索图像中找到的瞄点的像素坐标值
	//para tcp input 拍摄图片时TCP的xy坐标值(单位mm)
	//para z input 瞄点所在平面相对参考平面的高度(单位mm)
	cv::Point2f getXY(const cv::Point2f pointInImg, const cv::Point2f tcp, double z); 
	//标定参数
	bool readParameter(const string filePath);
	//保存参数
	static bool writeParameter(const string filePath, SensorPara &para);
	//标定激光条纹成像的平移量和世界坐标Z轴移动量的比例关系
	//laserImg中需有四张激光图像，第一张为参考位置处的激光条纹图像，
	double calibrateZtoLaser(vector <cv::Mat> &laserImg, const vector<double> zdis);
	//标定求解单应矩阵所需的对应关系
	void ratioxyFromZ(const vector<cv::Mat> hmats, const vector<double> z, double &ratio, cv::Point2d &dir);
	
public:
	//根据Z轴的值（相对参考位置）生成从相机到当前Z轴平面的单应矩阵
	cv::Mat getHmat(double Z);
private:
	bool m_isCalibrated;
	SensorPara m_para;
	LaserExtractPara m_laserPara;
	string m_errInf;
};

//基于ransac的平面直线拟合(还不是很稳定)
bool fitLine_ransac(const vector<Point2f> &srcPnts, double minIneRatio, double disThreasold, double minDis, vector <Vec3f> &lines);


//从直线上的一点和直线的方向向量获得点法式直线方程，已归一化
Vec3f getline_pd(const Point2f &pt, const Point2f &dir);

//从直线上的两点直线方程，方程为点法式
//返回值line 为大小为3的向量，分别为Ax+By+C = 0的三个参数，A,B已经归一化
Vec3f getline_2p(const Point2f &p1, const Point2f &p2);

//绘制由点法式表示的直线
void drawLine(Mat &src, const Vec3f &line);

