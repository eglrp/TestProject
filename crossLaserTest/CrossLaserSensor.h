
#pragma once
#include "Scanner.h"

class abstractCamera;
class CrossLaserSenorAlgorithm;
class CrossLaserSensor:public abstractScanner
{
	//扫描头触发模式
	enum TriggerMode_s
	{
		TimeMode = 0,
		EncoderMode,
		ExternalInputMode,
		ManualMode,
	};
	//扫描头设置参数
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
	//扫描
	//note: timeout在此处没有用
	virtual bool Scan(std::vector<cv::Point3f> &pointCloud, int timeout = -1);
	//初始化设备
	bool initializeDevice(const std::string paraFilePath, LaserDirection laserDir);
	//保存参数配置文件
	bool saveSetCfg(const std::string setFilePath);

	CrossLaserSenorAlgorithm *remodeler; //重建对象
	abstractCamera *camera; //相机指针，定义为抽象相机，以备快速更换不同品牌的相机
	laserScannerSetting *scannerSet;


};

