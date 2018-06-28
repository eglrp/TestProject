#include "CrossLaserSenorAlgorithm.h"
#include "CoreAlgorithm.h"

CrossLaserSenorAlgorithm::CrossLaserSenorAlgorithm():
isCalibrated(false),
m_errInf("no error")
{
}


CrossLaserSenorAlgorithm::~CrossLaserSenorAlgorithm()
{
}


bool CrossLaserSenorAlgorithm::estimateIntrinsicPara(const CamPara &camPara, const vector<RT> &targetPlaneRT1, const vector<RT> &targetPlaneRT2,
	const vector<points_2d> &stripes1, const vector<points_2d> &stripes2)
{
	int _n1 = targetPlaneRT1.size();
	int _n2 = targetPlaneRT2.size();
	if (_n1 != stripes1.size() || _n2 != stripes2.size() || !_n1 || !_n2){
		m_errInf = "input error in estimateIntrinsicPara()";
		return false;
	}

	//***将条纹中心点投影到约束平面
	vector <cv::Point3f> pnt3d1, pnt3d2;
	cv::Mat intrinsic, diff;
	camPara.cvtCamPara(intrinsic, diff);
	for (int i = 0; i < _n1; i++){
		vector<cv::Point3f> _tmpnt;
		CoreAlgorithm::project2CalibrationBoard(stripes1[i], _tmpnt, intrinsic, diff, targetPlaneRT1[i].R,targetPlaneRT1[i].T);
		std::copy(_tmpnt.begin(), _tmpnt.end(), back_inserter(pnt3d1));
	}
	for (int i = 0; i < _n2; i++){
		vector<cv::Point3f> _tmpnt;
		CoreAlgorithm::project2CalibrationBoard(stripes2[i], _tmpnt, intrinsic, diff, targetPlaneRT2[i].R, targetPlaneRT2[i].T);
		std::copy(_tmpnt.begin(), _tmpnt.end(), back_inserter(pnt3d2));
	}

	//***平面拟合获取两个刀光平面参数
	double plane1[4] = { 0 }, plane2[4] = { 0 };
	CAlgorithm::FitPlane(pnt3d1, plane1);
	CAlgorithm::FitPlane(pnt3d2, plane2);

	//***标定
	if (!estimateIntrinsicPara(camPara, plane1, plane2)) return false;
	m_camPara = camPara;
	isCalibrated = true;
	return true;
}

bool CrossLaserSenorAlgorithm::estimateIntrinsicPara(const CamPara &camPara, double *planePara1, double *planePara2)
{
	if (planePara1 == NULL || planePara2 == NULL) {
		m_errInf = "plane parameter is null";
		return false;
	}
	cv::Mat camFrameInLaserFrame1, camFrameInLaserFrame2;
	CAlgorithm::CaculateHMatric(planePara1, camPara, camFrameInLaserFrame1, m_transMat1);
	CAlgorithm::CaculateHMatric(planePara2, camPara, camFrameInLaserFrame2, m_transMat2);

	//***建立十字激光传感器坐标系：两激光平面的交线定义为Z轴；1号平面的法相作为y轴；原点为相机光心在光刀平面交线上的投影
	cv::Vec3d y(planePara1[0], planePara1[1], planePara1[2]);
	if (y.dot(cv::Vec3d(0, 0, 1)) > 0){ //保证y轴只想光心，
		y = -y;
	}
	cv::Vec3d n2(planePara2[0], planePara2[1], planePara2[2]);
	cv::Vec3d z = y.cross(n2);
	if (z.dot(cv::Vec3d(0, 0, 1)) < 0){ //保证Z的朝向和相机Z轴同向
		z = -z; 
	}
	cv::Vec3d x = y.cross(z);

	//光心在交线的投影
	cv::Mat t = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat A = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat d = (cv::Mat_<double>(3, 1) << -planePara1[3], -planePara2[3], 0);
	A.row(0) = cv::Mat(y);
	A.row(1) = cv::Mat(n2);
	A.row(2) = cv::Mat(z);
	cv::invert(A, A);
	t = A*d;
	cv::Mat rtInCam = cv::Mat::eye(4, 4, CV_64FC1);
	rtInCam.row(0).colRange(0, 3) = cv::Mat(x);
	rtInCam.row(1).colRange(0, 3) = cv::Mat(y);
	rtInCam.row(2).colRange(0, 3) = cv::Mat(z);
	rtInCam.rowRange(0,3).col(3) = t;
	cv::invert(rtInCam, rtInCam);

	//***计算两条条纹到传感器坐标系的变换矩阵
	cv::Mat laser1InCam, laser2InCam;
	cv::invert(camFrameInLaserFrame1, laser1InCam);
	cv::invert(camFrameInLaserFrame2, laser2InCam);
	rt1 = rtInCam* laser1InCam;
	rt2 = rtInCam *laser2InCam;
	return true;
}

bool CrossLaserSenorAlgorithm::remodel(const cv::Mat &src, vector<cv::Point3f> &pointCloud, const cv::Rect *ROI)
{

	return false;
}