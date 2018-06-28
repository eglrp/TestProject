//����ɨ���ǽӿڵĳ�����
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
	//ɨ��
	//pointCloud output ��������
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
	//������Ϣ
	std::string err_inf;
	//�豸ID
	std::string ID;
	//�Ƿ��Ѿ���ʼ��
	bool hadInitialied;
	//�Ƿ�����ɨ��
	bool Scanning;
};

