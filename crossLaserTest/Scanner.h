//定义扫描仪接口的抽象类
#pragma once
#include <string>
#include <vector>
#include <cv.h>
enum LaserDirection
{
	COL = 0,
	ROW,
};

class  abstractScanner
{
public:
	abstractScanner() :hadInitialied(false),
		err_inf(""),
		Scanning(false){}
	virtual ~abstractScanner(){}
	//扫描
	//pointCloud output 点云数据
	virtual bool Scan(std::vector<cv::Point3f> &pointCloud, int timeout = -1) = 0;

	bool isInitializated(){
		return hadInitialied;
	}
	std::string getError(){
		return err_inf;
	}
	std::string getScannerID(){
		return ID;
	}
	bool isScanning(){
		return Scanning;
	}

protected:
	//错误信息
	std::string err_inf;
	//设备ID
	std::string ID;
	//是否已经初始化
	bool hadInitialied;
	//是否正在扫描
	bool Scanning;
};

