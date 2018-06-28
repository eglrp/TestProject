#pragma once
class CRobotControl
{
public:
	CRobotControl();
	~CRobotControl();
public:
	virtual bool ConnectRobot() = 0;
};

