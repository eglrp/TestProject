//ʮ�ּ��⴫�����������㷨����Ӳ���޹�
#pragma once
#include <vector>
#include <cv.h>
#include "Algorithm.h"
using namespace std;

	//ʮ�ּ��⴫�����㷨ʵ��
class CrossLaserSenorAlgorithm
	{
		typedef vector<cv::Point2f> points_2d;
		friend class CrossLaserSensor;
		friend class ScannerCalibrate;
	public:
		CrossLaserSenorAlgorithm();
		~CrossLaserSenorAlgorithm();
		//���봫�����ڲ�
		bool readIntrinsicPara(const string &fileName);
		//�궨ʮ�ּ��⴫�������ڲμ����궨ʮ�ּ�����������ƽ���λ��
		//note:�����������Ĳ���ҪԤ�Ƚ���У�����������ڻ�У����
		bool estimateIntrinsicPara(const CamPara &camPara, const vector<RT> &targetPlaneRT1, const vector<RT> &targetPlaneRT2, 
			const vector<points_2d> &stripes1, const vector<points_2d> &stripes2);
		//�궨ʮ�ּ��⴫�������ڲμ����궨ʮ�ּ�����������ƽ���λ��
		bool estimateIntrinsicPara(const CamPara &camPara, double *planePara1, double *planePara2);
		//�ɼ���ͼ����������������ĵ���ά����ϵ������������ϵ�£�
		bool remodel(const cv::Mat &src, vector<cv::Point3f> &pointCloud, const cv::Rect *ROI = NULL);
		
	protected:
		CamPara m_camPara; //�����������
		cv::Mat m_transMat1, m_transMat2; //��ͼ������ϵ�������⵶ƽ��������ת������
		string m_errInf; //������Ϣ
	private:
		bool isCalibrated; //�Ƿ��Ѿ��궨��ʶ��
		cv::Mat map1, map2; //ͼ��������ӳ�����
		cv::Mat gausk; //��˹�˲���
		cv::Mat rt1, rt2;//������ƽ������ϵ�ڴ���������ϵ�µ�λ�˾��� //todo::��m_transMat1, m_transMat2���Ժϲ���һ������
	};


