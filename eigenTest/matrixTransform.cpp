#include "matrixTransform.h"

cv::Mat QuaternionToR33(double *quaternion)
{
	cv::Mat tep;
	return tep;
}

Eigen::Quaterniond R33ToQuaternion(double *rt33)
{
	Eigen::Matrix3d tp(rt33);
	Eigen::Quaterniond qd(tp);
	return qd;
}

Eigen::Quaterniond R33ToQuaternion(cv::Mat_<double> rt33)
{
	Eigen::Matrix3d tp;
	cv::cv2eigen(rt33.t(), tp);
	Eigen::Quaterniond qd(tp);
	return qd;
}

cv::Mat_<double> transl(double x, double y, double z)
{
	Eigen::Affine3d trans = Eigen::Affine3d::Identity();
	trans(0, 3) = x;
	trans(1, 3) = y;
	trans(2, 3) = z;
	//trans.translate(Eigen::Vector3d(x, y, z));//这种方式会报错
	cv::Mat_<double> _mat;
	cv::eigen2cv(trans.matrix(), _mat);
	return _mat;
}

cv::Mat_<double> trotz(double angle)
{
	Eigen::Transform<double, 3, Eigen::Affine> t(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
	//note:以下方式是错误的。
	//Eigen::Transform<double, 3, Eigen::Affine> t;
	//t = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
	cv::Mat_<double> _mat(4, 4, t.data());
	return _mat.t();
}

cv::Mat_<double> troty(double angle)
{
	Eigen::Transform<double, 3, Eigen::Affine> t(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
	cv::Mat_<double> _mat(4, 4, t.data());
	return _mat.t();
}

cv::Mat_<double> trotx(double angle)
{
	Eigen::Transform<double, 3, Eigen::Affine> t(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));
	cv::Mat_<double> _mat(4, 4, t.data());
	return _mat.t();
}