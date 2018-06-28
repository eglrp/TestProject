#pragma once
#include "SharedHead.h"
#include <iostream>
#include <fcntl.h>
#include <io.h>
//#include <afxwin.h>
#include <windows.h>
#include <fstream>

//boost serialization
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

class cvtool
{
public:
	cvtool();
	~cvtool();
public:
	//解析相机参数
	static void cvtCamPara(const CamPara& camParas, Mat& intrinsicPara, Mat& distCoeffs);
	//获取角点的世界坐标系值
	static void getRingCenterInWorld(const int ringRow, const int ringCol, const int distanceX, const int distanceY,
		const int GroupSize, vector<vector<Point3f>> &outPnts);
	//获取角点的世界坐标系值 重载 
	static void getRingCenterInWorld(const int ringRow, const int ringCol, const int distanceX, const int distanceY,
		vector<Point3f> &outPnts);
	//读取txt点云数据
	static bool readPointCloudFormTxt(const string filename, vector<Point3f> &pnts);
	//利用内存映射和线程池读取txt点云文件
	static bool readPointCloudFromTxt_fast(const string filename, vector<Point3f> &pnts);
	//保存txt点云数据
	static bool writePointCloudFormTxt(const string filename, vector<Point3f> &pnts);
	//cai测试txt读取速度方法(狂快)
	static bool readfileTest(const string filename, vector<Point3f> &pnts);
	//cai测试txt保存速度方法(狂快)
	static bool writefileTest(const string filename, vector<Point3f> &pnts);

	//序列化和反序列化mat（通常函数末尾为A表可以双向系列化）
	template<class Archive>
	static void cvmatSerializeA(Archive &ar, cv::Mat& mat, const unsigned int version)
	{
		int cols, rows, type;
		bool continuous;

		if (Archive::is_saving::value) {
			cols = mat.cols; rows = mat.rows; type = mat.type();
			continuous = mat.isContinuous();
		}

		ar & cols & rows & type & continuous;

		if (Archive::is_loading::value)
			mat.create(rows, cols, type);

		if (continuous) {
			const unsigned int data_size = rows * cols * mat.elemSize();
			ar & boost::serialization::make_array(mat.ptr(), data_size);
		}
		else {
			const unsigned int row_size = cols*mat.elemSize();
			for (int i = 0; i < rows; i++) {
				ar & boost::serialization::make_array(mat.ptr(i), row_size);
			}
		}
	}
};

