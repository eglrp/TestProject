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
static float pulseDis = 1; //�豸Z��һ�������Ӧ������ߴ磬��λ��mm
//��ȡ�豸Z�������
int getZInDevice(double Z);

class laserSensor
{
public:
	struct LaserExtractPara{
		LaserExtractPara();
		int threasold; //������ǰ���Աȶ���ֵ
		int windowWidth; //���ƿ��Ԥ��ֵ
		cv::Mat guassK; //��˹��

		//���л��ͷ����л��ṹ��
		template<class Archive>
		void serializeA(Archive& ar, const unsigned int version){
			ar & threasold;
			ar & windowWidth;
			cvtool::cvmatSerializeA(ar, guassK, version);
		}
	};
	//�豸�궨����
	struct SensorPara{
		SensorPara();
		double ratio; //ƽ�м�������ͼ�������ʵ�ʸ߶ȱ仯�ı�������
		double refLaserLine[3]; //�ο�ƽ��ļ�������ֱ�߲��� ax + by + c = 0�е�a b c,ע�⣺a,b���������ֱ�ߴ�ֱ��������
		double hpara[3]; //��Ӧ��������������һ��ΪZ���ƶ��������ر仯���ı������ӣ�2��3Ϊxy�仯����
		cv::Mat refHmat; //�ο�λ�ô��ĵ�Ӧ�Ծ���
		cv::Mat worldInHand; //���������xyƽ����TCP����ϵ��xyƽ��ı任����ֻ���������������Z����豸Z��ƽ�е������
		LaserExtractPara laserExtractPara;//����������ȡ����
		//test 
		double polypara[3];

		//���л��ͷ����л��ṹ��
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

	//��ȡ����ָʾ�ĸ߶�(��Բο�λ��)
	double getZ(cv::Mat &src, const cv::Rect &ROI = cv::Rect(0,0,0,0));
	//��ͼ����ȡ�õ����豸xy����ϵ�µ�λ��
	//para pointInImg input ������ͼ�����ҵ���������������ֵ
	//para tcp input ����ͼƬʱTCP��xy����ֵ(��λmm)
	//para z input �������ƽ����Բο�ƽ��ĸ߶�(��λmm)
	cv::Point2f getXY(const cv::Point2f pointInImg, const cv::Point2f tcp, double z); 
	//�궨����
	bool readParameter(const string filePath);
	//�������
	static bool writeParameter(const string filePath, SensorPara &para);
	//�궨�������Ƴ����ƽ��������������Z���ƶ����ı�����ϵ
	//laserImg���������ż���ͼ�񣬵�һ��Ϊ�ο�λ�ô��ļ�������ͼ��
	double calibrateZtoLaser(vector <cv::Mat> &laserImg, const vector<double> zdis);
	//�궨��ⵥӦ��������Ķ�Ӧ��ϵ
	void ratioxyFromZ(const vector<cv::Mat> hmats, const vector<double> z, double &ratio, cv::Point2d &dir);
	
public:
	//����Z���ֵ����Բο�λ�ã����ɴ��������ǰZ��ƽ��ĵ�Ӧ����
	cv::Mat getHmat(double Z);
private:
	bool m_isCalibrated;
	SensorPara m_para;
	LaserExtractPara m_laserPara;
	string m_errInf;
};

//����ransac��ƽ��ֱ�����(�����Ǻ��ȶ�)
bool fitLine_ransac(const vector<Point2f> &srcPnts, double minIneRatio, double disThreasold, double minDis, vector <Vec3f> &lines);


//��ֱ���ϵ�һ���ֱ�ߵķ���������õ㷨ʽֱ�߷��̣��ѹ�һ��
Vec3f getline_pd(const Point2f &pt, const Point2f &dir);

//��ֱ���ϵ�����ֱ�߷��̣�����Ϊ�㷨ʽ
//����ֵline Ϊ��СΪ3���������ֱ�ΪAx+By+C = 0������������A,B�Ѿ���һ��
Vec3f getline_2p(const Point2f &p1, const Point2f &p2);

//�����ɵ㷨ʽ��ʾ��ֱ��
void drawLine(Mat &src, const Vec3f &line);

