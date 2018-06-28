#include "cvtool.h"


cvtool::cvtool()
{
}


cvtool::~cvtool()
{
}

void cvtool::cvtCamPara(const CamPara& camParas, Mat& intrinsicPara, Mat& distCoeffs)
{
	intrinsicPara.create(3, 3, CV_64FC1);
	distCoeffs.create(1, 4, CV_64FC1);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			intrinsicPara.at<double>(i, j) = camParas.CameraIntrinsic[i][j];
		}
	}
	for (int i = 0; i < 4; i++)
	{
		distCoeffs.at<double>(0, i) = camParas.DistortionCoeffs[i];
	}
}

void cvtool::getRingCenterInWorld(const int ringRow, const int ringCol, const int distanceX, const int distanceY,
	const int GroupSize, vector<vector<Point3f>> &outPnts)
{
	for (int i = 0; i < GroupSize; ++i)
	{
		vector<Point3f> point3fsTem;
		Point3f pntTem;
		pntTem.z = 0;
		for (int row = 0; row < ringRow; ++row)
		{
			pntTem.y = row * distanceY;
			for (int col = 0; col < ringCol; ++col)
			{
				pntTem.x = col * distanceX;
				point3fsTem.push_back(pntTem);
			}
		}
		outPnts.push_back(point3fsTem);
	}
}
void cvtool::getRingCenterInWorld(const int ringRow, const int ringCol, const int distanceX, const int distanceY,
	 vector<Point3f> &outPnts)
{
	Point3f pntTem;
	pntTem.z = 0;
	for (int row = 0; row < ringRow; ++row)
	{
		pntTem.y = row * distanceY;
		for (int col = 0; col < ringCol; ++col)
		{
			pntTem.x = col * distanceX;
			outPnts.push_back(pntTem);
		}
	}
}

bool cvtool::readPointCloudFormTxt(const string filename, vector<Point3f> &pnts)
{
	double t2 = cv::getTickCount();
	ifstream fi;
	fi.open(filename);
	if (!fi.is_open())
	{
		std::cout << "��ȡ�ļ�����" << endl;
		return false;
	}
	t2 = (cv::getTickCount() - t2) / cv::getTickFrequency();
	std::cout << "���ļ���ʱ��" << t2 << " sec" << endl;
	pnts.clear();
	double t1 = cv::getTickCount();
	while (!fi.eof())
	{
		Point3f _pnt;
		fi >> _pnt.x >> _pnt.y >> _pnt.z;
		pnts.push_back(_pnt);
	}
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	std::cout << "��ȡ�ļ���ʱ��" << t1 << " sec" << endl;
	return true;
}

bool cvtool::readPointCloudFromTxt_fast(const string filename, vector<Point3f> &pnts)
{

#if 0 //ʱ�䲢û������������
	ifstream fi;
	string buffer;
	fi.open(filename);
	if (!fi.is_open())
	{
		cout << "��ȡ�ļ�����" << endl;
		return false;
	}
	buffer.assign(istreambuf_iterator<char>(fi), istreambuf_iterator<char>());
	stringstream strStr;
	strStr.str(buffer);
	pnts.clear();
	double t1 = cv::getTickCount();
	while (!strStr.eof())
	{
		Point3f _pnt;
		strStr >> _pnt.x >> _pnt.y >> _pnt.z;
		pnts.push_back(_pnt);
	}
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	cout << "��ȡ�ļ���ʱ��" << t1 << " sec" << endl;
	return true;
#endif
	
	//����ļ����
	WCHAR wszClassName[256];
	memset(wszClassName, 0, sizeof(wszClassName));
	MultiByteToWideChar(CP_ACP, 0, filename.c_str(), strlen(filename.c_str()) + 1, wszClassName,
		sizeof(wszClassName) / sizeof(wszClassName[0]));

	HANDLE hFile = CreateFile(
		wszClassName,   //�ļ���
		GENERIC_READ | GENERIC_WRITE, //���ļ����ж�д����
		FILE_SHARE_READ | FILE_SHARE_WRITE,
		NULL,
		OPEN_EXISTING,  //���Ѵ����ļ�
		FILE_ATTRIBUTE_NORMAL,
		0);

	//����ֵsize_high,size_low�ֱ��ʾ�ļ���С�ĸ�32λ/��32λ
	DWORD size_low, size_high;
	size_low = GetFileSize(hFile, &size_high);

	//�����ļ����ڴ�ӳ���ļ���   
	HANDLE hMapFile = CreateFileMapping(
		hFile,
		NULL,
		PAGE_READWRITE,  //��ӳ���ļ����ж�д
		size_high,
		size_low,   //������������64λ������֧�ֵ�����ļ�����Ϊ16EB
		NULL);
	if (hMapFile == INVALID_HANDLE_VALUE)
	{
		std::cout << "Can't create file mapping.Error " << GetLastError << endl;
		CloseHandle(hFile);
		return false;
	}

	//���ļ�����ӳ�䵽���̵ĵ�ַ�ռ�
	void* pvFile = MapViewOfFile(
		hMapFile,
		FILE_MAP_READ | FILE_MAP_WRITE,
		0,
		0,
		0);
	char *p = (char*)pvFile;
	stringstream str;
	str.str(p);
	double t1 = cv::getTickCount();
	while (!str.eof())
	{
		Point3f _pnt;
		str >> _pnt.x >> _pnt.y >> _pnt.z;
		pnts.push_back(_pnt);
	}
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	std::cout << "��ȡ�ļ���ʱ��" << t1 << " sec" << endl;

	////����ӳ��
	//UnmapViewOfFile(pvFile);
	////�ر��ļ�
	//CloseHandle(hFile);
	return true;
}

bool cvtool::writePointCloudFormTxt(const string filename, vector<Point3f> &pnts)
{
	double t1 = cv::getTickCount();
	fstream pointCloud(filename, ios_base::out);
	for (int i = 0; i < pnts.size(); ++i)
	{
		pointCloud << pnts[i].x << " " << pnts[i].y << " " << pnts[i].z << endl;
	}
	pointCloud.close();
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	std::cout << "�����ļ���ʱ��" << t1 << " sec" << endl;
	return true;
}

bool cvtool::readfileTest(const string filename, vector<Point3f> &pnts)
{
	//����1��һ�ζ����ڴ�
#if 1
	double t1 = cv::getTickCount();
	filebuf *pbuf;
	ifstream fileread;
	long size;
	char * buffer;
	// Ҫ���������ļ���������ö����ƴ�   
	fileread.open(filename, ios::binary);
	// ��ȡfilestr��Ӧbuffer�����ָ�루�������������ָ�룩
	pbuf = fileread.rdbuf();

	// ����buffer���󷽷���ȡ�ļ���С  ������λλ��ָ��ָ���ļ���������ĩβʱ��pubseekoff���ص�ֵ���������ļ�����С��
	size = pbuf->pubseekoff(0, ios::end, ios::in);
	pbuf->pubseekpos(0, ios::in);   //����pbufָ���ļ�����ʼλ��

	// �����ڴ�ռ�  
	buffer = new char[size];

	// ��ȡ�ļ�����  
	pbuf->sgetn(buffer, size); 

	fileread.close();
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	
	// �������׼���  
	double t2 = cv::getTickCount();
	//cout.write(buffer, size);

//��ѷ������ַ���������buffer
	string temp = "";
	Point3f Pnts;

	float x = 0;
	float y = 0;
	float z = 0;
	bool isy = false;
	while (*buffer != '\0')
	{
		if (*buffer != '\n' && *buffer != '\r')
		{
			if (*buffer != ' ')
			{
				temp += *buffer;
			}
			else
			{
				if (!isy)  //�����x��ֵ
				{
					if (!temp.empty())
					{
						isy = !isy;
						sscanf(temp.data(), "%f", &x);
						Pnts.x = x;
						temp = "";
					}
				}
				else                  //�����y��ֵ
				{
					if (!temp.empty())
					{
						isy = !isy;
						sscanf(temp.data(), "%f", &y);
						Pnts.y = y;
						temp = "";
					}
				}
			}
		}
		else   //������z
		{
			if (!temp.empty())
			{
				sscanf(temp.data(), "%f", &z);
				Pnts.z = z;
				temp = "";
				pnts.push_back(Pnts);
			}
		}
		buffer++;
	}
	t2 = (cv::getTickCount() - t2) / cv::getTickFrequency();
	//delete[] buffer;  //�ͷŻ�������ͷ�û���⣬�����ǶѺ�ջ������
	std::cout << "txt�����ڴ棺" << t1 << " sec" << endl;
	std::cout << "txtת�����ݣ�" << t2 << " sec" << endl;
	//�����ַ��ָ�
#if 0
	char *Surplus = strtok(buffer, " ");
	while (Surplus)
	{
		//�жϵ�ǰ��ȡƬ�����޻س����Ǿͷָ�
		string str1 = Surplus;
		
		//stringתchar
		char str_char1[20];
		str1.copy(str_char1, 20, 0); //����5�������Ƽ����ַ���0�����Ƶ�λ��
		//*(str_char1 + 20) = '/0';
		

		char key = '\r\n';  //ָ��һ���ַ�
		char *strtemp;
		strtemp = strchr(str_char1, key);
		if (strtemp != NULL) //����ҵ��лس�
		{
			//ȥ���س�
			char *enter = strtok(str_char1, "\r\n");
			//stringתchar
			string str2 = enter;
			char str_char2[20];
			str2.copy(str_char2, 20, 0); //����5�������Ƽ����ַ���0�����Ƶ�λ��
			//*(str_char2 + 20) = '/0';

			double z = atof(str_char2);
			printf("%.5f\n", z);

			enter = strtok(NULL, "\r\n");
			//stringתchar
			string str3 = enter;
			char str_char3[20];
			str3.copy(str_char3, 20, 0); //����5�������Ƽ����ַ���0�����Ƶ�λ��
			//*(str_char3 + 20) = '/0';
		
			double x = atof(str_char3);	
			printf("%.5f\n", x);
		}
		else
		{
			double num = atof(str_char1);
			printf("%.5f\n", num);
		}
		Surplus = strtok(NULL, " ");
	}
#endif	
	


#endif
	//����2��һ���ж�ȡtxt�ļ���ʱ��̫���ˣ�ƽ��һ����Ҫ0.00015s
#if 0
	double t1 = cv::getTickCount();
	string line;
	int n = 0;
	char *Surplus;
	ifstream fp(filename);//,ios::binary);
	while (!fp.eof())
	{
		double t2 = cv::getTickCount();
		//ÿ��������ʱ��
		getline(fp, line);


		//getlinr+stroke+vector<string>
#if 0
			vector<string> elems;
			char line_char[40];
			line.copy(line_char, 40, 0); //����5�������Ƽ����ַ���0�����Ƶ�λ��
			char *s = strtok(line_char, " ");
			while (s != NULL) {
				elems.push_back(s);
				s = strtok(NULL, " ");
			}
			Point3f Pnts;
			Pnts.x = atof(elems[0]);


#endif

		//getline+strtok+stringתchar 
#if 0
		char line_char[40];
		line.copy(line_char, 40, 0); //����5�������Ƽ����ַ���0�����Ƶ�λ��
		//*(line_char + 20) = '/0';   //�ַ�������ֶ���β������ûʲôӰ��
		Point3f Pnts;
		Surplus = strtok(line_char, " ");
		Pnts.x = atof(Surplus);
		//printf("%.5f\n", Pnts.x);
		Surplus = strtok(NULL, " ");
		Pnts.y = atof(Surplus);
		//printf("%.5f\n", Pnts.y);
		Surplus = strtok(NULL, "");
		Pnts.z = atof(Surplus);
		//printf("%.5f\n", Pnts.z);
		pnts.push_back(Pnts);
#endif
		n++;
		t2 = (cv::getTickCount() - t2) / cv::getTickFrequency();
		cout << "��ǰ�㸳ֵʱ�䣺 " << t2 << " sec����ǰ�ǵ�" <<n<<"����"<< endl;
	}
	delete[] Surplus;
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	cout << "��ȡtxtʱ�䣺 " << t1 << " sec" << endl;
#endif

	return true;
}

bool cvtool::writefileTest(const string filename, vector<Point3f> &pnts)
{
	//1.�������Ű�x y z��ʽת����һ�������ַ���buffer
	double t1 = cv::getTickCount();
	//��ssֱ�Ӷ����е����֣�һ��ת��string���ڶ�ȡ����ʱsize��С����
	stringstream ss;
	for (int i = 0; i < pnts.size();i++)
	{
		ss << pnts[i].x << " " << pnts[i].y << " " << pnts[i].z << "\r\n";
	}
	string buffer(ss.str());
	//sprintf����ת�ַ���

	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	//2.��һ������bufferһ��д��������ļ�
	double t2 = cv::getTickCount();
	ofstream filewrite;
	filewrite.open(filename, ios::binary);
	filewrite << buffer;
	filewrite.close();
	t2 = (cv::getTickCount() - t2) / cv::getTickFrequency();
	std::cout << "txtת�����ݣ�" << t1 << " sec" << endl;
	std::cout << "txtд���ڴ棺" << t2 << " sec" << endl;
	return true;
}
