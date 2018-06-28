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
	//�����������
	static void cvtCamPara(const CamPara& camParas, Mat& intrinsicPara, Mat& distCoeffs);
	//��ȡ�ǵ����������ϵֵ
	static void getRingCenterInWorld(const int ringRow, const int ringCol, const int distanceX, const int distanceY,
		const int GroupSize, vector<vector<Point3f>> &outPnts);
	//��ȡ�ǵ����������ϵֵ ���� 
	static void getRingCenterInWorld(const int ringRow, const int ringCol, const int distanceX, const int distanceY,
		vector<Point3f> &outPnts);
	//��ȡtxt��������
	static bool readPointCloudFormTxt(const string filename, vector<Point3f> &pnts);
	//�����ڴ�ӳ����̳߳ض�ȡtxt�����ļ�
	static bool readPointCloudFromTxt_fast(const string filename, vector<Point3f> &pnts);
	//����txt��������
	static bool writePointCloudFormTxt(const string filename, vector<Point3f> &pnts);
	//cai����txt��ȡ�ٶȷ���(���)
	static bool readfileTest(const string filename, vector<Point3f> &pnts);
	//cai����txt�����ٶȷ���(���)
	static bool writefileTest(const string filename, vector<Point3f> &pnts);

	//���л��ͷ����л�mat��ͨ������ĩβΪA�����˫��ϵ�л���
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

