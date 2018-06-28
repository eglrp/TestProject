#pragma once
#include "SharedHead.h"
#include "cvtool.h"
#include <time.h>
#include <queue>
#define  PI 3.141592654
class CAlgorithm
{
public:
	//�����������ģ�index��ʾ�����������/��������pt��ʾ��ȡ�����������ĵ��ڵ�ǰ��/�е�������λ�á�
	struct laserCenter
	{
		int index;
		vector <float> pt;
	};
	typedef vector< vector<Point2f> > clusterPnts; //ƽ��㼯

public:
	//TODO:����д�ļ����㷨
	//�����ϵľ�γ�����Ѿ��뾶�������λ��
	static bool getBallCenter(const vector<Point3f>srcPnts, const double ridus, Point3f &BallCenter);
	//��д�µĿռ�ƽ������㷨
	static bool FitPlane2(vector<Point3f>srcPnts, double *PlaneParam); //ʵ���з������е�ƽ������㷨��geometric����Ͻ����Щ����

	//�������ͼ�������͹���ƽ������ϵ�µ����������ĵ�Ӧ����
	//note: Hmat�������ֵͨ����С��ֱ�ӱ��棬���¶�ȡʱ��ֵ���Ȳ���������������ٱ��档
	static bool CaculateHMatric(const double *planeParams, const CamPara &camParam, cv::Mat &camFrameInLaserFrame, cv::Mat &Hmat);
	//end todo
	//�����ߵķ�������
	//�ռ�Բ���
	static bool FitCircle3D(vector<Point3f>srcPnts, circle3d &circle3dParam);
	//�ռ�ƽ����С�������
	static bool FitPlane(vector<Point3f>srcPnts, double *PlaneParam);
	//����ͶӰ������������ϵ�µ���ά��ͶӰ��ͼ������ϵ�У����ó�ͶӰ���
	static double projectPointToImg(const vector<Point3f> &srcPnts, const vector<Point2f> &srcImgPnts,
		const Mat& intrinsicPara, const Mat& distCoeffs, const Mat &r, const Mat &t, vector<Point2f> &dstPnt);
	//��������Ҷ�����
	static bool getRowMaxLocMid(Mat& rowMat, int colStart, int colEnd, int& maxValue, int& colMaxMid, const bool& MaxMidType);
	//����������Ҷ�����,����Ϊ��ָ��
	static bool getRowMaxLocMid(uchar* rowPtr, int colStart, int colEnd, int& maxValue, int& colMaxMid, const bool& MaxMidType);
	//�������ǰ�������Ҷ�����,����Ϊ��ָ��
	static bool getMultMax(uchar* rowPtr, int colStart, int colEnd, int windowwidth, int findNum, vector <uchar>& maxValue, vector <int>& colMax);
	//��ȡ�����ֵ��ͬ��ȡ�����ֵ�Ĳ�����ʽ������ͬ�����������ĵ�ַ���в���
	static bool getColMaxLocMid(Mat& colMat, int rowStart, int rowEnd, int& maxValue, int& rowMaxMid, const bool& MaxMidType);
	//��˹��
	static bool GaussKNew(Mat& gaussK, int Krow, int Kcol, float Ksig);
	//���ָ�˹�˲�
	static bool PartGaussianBlur(Mat& srcImg, Mat& srcCpy, int rowCurrent, int colStart, int colEnd, Mat& gaussK);
	//��ͼ�񲿷ָ�˹�˲�
	static bool PartGaussianBlur_col(Mat& srcImg, Mat& srcCpy, int curCol, int rowStart, int rowEnd, Mat& gaussK);
	//��������ͶӰ����ͼ���ͶӰ����������ϵ������pnpλs�˹������
	//��������������ȡ�㷨�����з���ָ��roi�汾��
	static bool LaserCenterDetect_row(Mat& src, const int Threshold, const int windowwidth, int centerRange,
		vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI);
	//��������������ȡ�㷨�����з�����roi�汾��
	static  bool LaserCenterDetect_row(Mat& src, const int Threshold, const int windowwidth, int centerRange,
		vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType);
	//��������������ȡ�㷨����roi�汾��
	static bool LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange,
		vector<Point2f> &LaserCenter,double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType);
	////����������ȡ���б�����
	static bool LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange,
		vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI);

	//ȷ�����ƿ�ȡ�ǰ������������
	static bool getWTT(uchar *src, cv::Point pos, double &I1, double &I2, double &W);

	//�Ե��ƽ��и���仯
	static bool rigidTransformPointCould(const Mat& rigidTransformRT, vector<Point3f> &srcPnts, vector<Point3f> &dstPnts);
	//ͨ�������ڲ�ͬ����ϵ������ϵ����������ϵ֮��ĸ���任
	static void rigidTransformFrom3Pnt(const vector<Point3f>pntsInRef, const vector<Point3f> pntsInLocal, Mat &rt);
	//�����㹹��ֲ�����ϵ,�м����Ϊ����ϵԭ��
	static void frameFrom3Pnts(const vector<Point3f>pnts, Mat &rt);
	//����ϵ��Z����תang�ǵ���ת�任����
	static Mat rotz(double rotAngle,bool radFlag = false);
#if 0
	//����������ȡ 
	bool LaserCenterDetector(const Mat& srcimage, const cv::Rect mask, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter);
	//����������ȡ ��roi
	bool LaserCenterDetector(const Mat& srcimage, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter);
#endif

};

