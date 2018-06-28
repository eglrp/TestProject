// dwgToModel_3d.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h"
#include <iostream>

//dxf lib
#include "dl_dxf.h"
#include "dl_entities.h"
#include "dl_creationinterface.h"
#include "dl_creationadapter.h"
//pcl
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

using namespace std;



//将dxf平面视图拉伸成3D模型
bool dxfToModel3D(DL_Dxf srcDxf, pcl::PolygonMesh::Ptr dstModel)
{

}

int _tmain(int argc, char** argv)
{
	// Check given arguments:
	if (argc < 2) {
		cout << "参数有误！" << endl;
		return 0;
	}

	cout << "读入dxf文件：" <<argv[1]<< endl;
	DL_CreationAdapter* _adapter = new DL_CreationAdapter();
	DL_Dxf* dxf = new DL_Dxf();
	if (!dxf->in(argv[1], _adapter)) { // if file open failed
		std::cout << argv[1] << " could not be opened.\n";
		return;
	}

	cout << "拉伸并保存文件" << endl;
	_adapter->setExtrusion(0, 0, 1, 15);
	

	return 0;
}

