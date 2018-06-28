// dwgToModel_3d.cpp : �������̨Ӧ�ó������ڵ㡣
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



//��dxfƽ����ͼ�����3Dģ��
bool dxfToModel3D(DL_Dxf srcDxf, pcl::PolygonMesh::Ptr dstModel)
{

}

int _tmain(int argc, char** argv)
{
	// Check given arguments:
	if (argc < 2) {
		cout << "��������" << endl;
		return 0;
	}

	cout << "����dxf�ļ���" <<argv[1]<< endl;
	DL_CreationAdapter* _adapter = new DL_CreationAdapter();
	DL_Dxf* dxf = new DL_Dxf();
	if (!dxf->in(argv[1], _adapter)) { // if file open failed
		std::cout << argv[1] << " could not be opened.\n";
		return;
	}

	cout << "���첢�����ļ�" << endl;
	_adapter->setExtrusion(0, 0, 1, 15);
	

	return 0;
}

