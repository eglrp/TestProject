
#pragma once
#include "Scanner.h"

class abstractCamera;
class CrossLaserSenorAlgorithm;
class CrossLaserSensor:public abstractScanner
{
	//ɨ��ͷ����ģʽ
	enum TriggerMode_s
	{
		TimeMode = 0,
		EncoderMode,
		ExternalInputMode,
		ManualMode,
	};
	//ɨ��ͷ���ò���
	struct laserScannerSetting
	{
		TriggerMode_s mode = EncoderMode; //trigger mode	
		double exposureTime = 1; //exposure time,unit: ms
		int recontructionTimeOut = 1000; //recontrucition timeout
		int FrameRate = 140;//frame rate
		////time mode
		//double captureTime = 1000 / FrameRate; //trigger duration time, unit:s
		//double moveSpeed = 100; //scanner move speed on the y axis
		////encoder params
		//int encoderSpace = 350; //trigger pulse step 
		//double stepDis = 0.002; //distance per  pulse, unit:mm
		//int triggerNum = 1000; //trigger total number
		//int startPositon = 60000;//start triggger position	
		//MOVEDIR moveDir = POSITIVE; //move dir
	};
public:
	CrossLaserSensor(abstractCamera *camera);
	~CrossLaserSensor();
	//ɨ��
	//note: timeout�ڴ˴�û����
	virtual bool Scan(std::vector<cv::Point3f> &pointCloud, int timeout = -1);
	//��ʼ���豸
	bool initializeDevice(const std::string paraFilePath, LaserDirection laserDir);
	//������������ļ�
	bool saveSetCfg(const std::string setFilePath);

	CrossLaserSenorAlgorithm *remodeler; //�ؽ�����
	abstractCamera *camera; //���ָ�룬����Ϊ����������Ա����ٸ�����ͬƷ�Ƶ����
	laserScannerSetting *scannerSet;


};

