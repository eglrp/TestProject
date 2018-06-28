// eigenTest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "testFun.h"
#include "matrixTransform.h"


struct robTarget
{
	Eigen::Quaterniond rot; //旋转量
	Eigen::Vector3d transl; //平移量

	//相对本身平移
	void translate_local(const Eigen::Vector3d translation)
	{
		transl += translation;
	}

	//从4*4矩阵中解析出目标点
	void parseFromMatrix(const Eigen::Matrix4d matrix)
	{
		Eigen::Matrix3d _r(matrix.block(0, 0, 3, 3));
		rot = Eigen::Quaterniond(_r);
		transl = Eigen::Vector3d(matrix.block(0, 3, 3, 1));
	}

	//转化成4*4齐次矩阵
	Eigen::Matrix4d toMatrix4d()
	{
		Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
		mat.block(0, 0, 3, 3) = rot.toRotationMatrix();
		mat.block(0, 3, 3, 1) = transl.matrix();
		return mat;
	}

	//转化成字符串
	std::string  toString()
	{
		std::stringstream str;
		str << transl.x() << " " << transl.y() << " " << transl.z() << " " << rot.w() << " " << rot.x() << " " << rot.y() << " " << rot.z();
		return str.str();
	}
};

int _tmain(int argc, _TCHAR* argv[])
{
	//4*4的齐次变换矩阵
	float dat[16] = {0};
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	float *et = trans.data();
	trans = Eigen::Affine3f(Eigen::Matrix4f(dat));
	cout << trans.matrix() << endl;
	Eigen::Vector3f transl_e(trans.translation());
	Eigen::Quaternionf  qua(trans.rotation());
	cout << transl_e << endl;
	cout << qua.matrix() << endl;

#if 0
	//double rt33[9] = { 0.0078,   0.9999,    0.0070,
	//	0.9547, -0.0095,    0.2973, 
	//	0.2974,    0.0044, -0.9548 };
	double rt33[9] = { 0.646763892037881, - 0.762690283113672,	0,
		0.762690283113672,	0.646763892037881,	0,
		0,	0,	1
	};
	Eigen::Quaterniond d = R33ToQuaternion(rt33);
	cout << d.coeffs() << endl; //注意通过coeffs得到的四元素四个元素的顺序为：x,y,z,w

	//用数组给矩阵赋值
	double dat[16] = { 0,1 };
	dat[15] = 1;
	Eigen::Matrix4d _mat(dat);
	cout << _mat <<endl;

	double dat1[16] = { 1, 2, 3 };
	
	Eigen::Matrix4d _mat1(dat1);
	cout << "_mat1 = " << endl << _mat1 << endl;
	cout << _mat * _mat1<<endl;

	//4*4的齐次变换矩阵
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	float transl[3] = { 1, 2, 3 };
	Eigen::Vector3f transl_e(transl);
	trans.translate(transl_e);
	cout << trans.matrix()<<endl;

	//两矩阵相乘
#endif
	//transfrom between cv mat and eigen mat
#if 0
	cv::Mat rt = (cv::Mat_<double>(4, 4) << 1, 2, 3, 4, 1, 1, 2, 3, 344, 4, 5, 2, 0, 0, 0, 1);
	cv::Mat_<double> _rt(rt);
	cout << rt << endl;
	_rt(0, 1) = 3;
	cout << rt << endl;
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	Eigen::Matrix4f e_mat44;
	cv::cv2eigen(_rt, trans.matrix());
	cout << trans(0, 0) << trans(0, 1) << trans(0, 2) << endl;
	Eigen::Quaternion<double> df;
	Eigen::Transform<double, 3, Eigen::Affine> dfd;
	dfd.
#endif
#if 0
#endif
#if 0
	cout << trotz(3.1415);
	string  tre = "12;35;25;";
#endif

	return 0;
}