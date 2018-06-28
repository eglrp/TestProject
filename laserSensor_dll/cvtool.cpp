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
		std::cout << "读取文件错误！" << endl;
		return false;
	}
	t2 = (cv::getTickCount() - t2) / cv::getTickFrequency();
	std::cout << "打开文件耗时：" << t2 << " sec" << endl;
	pnts.clear();
	double t1 = cv::getTickCount();
	while (!fi.eof())
	{
		Point3f _pnt;
		fi >> _pnt.x >> _pnt.y >> _pnt.z;
		pnts.push_back(_pnt);
	}
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	std::cout << "读取文件耗时：" << t1 << " sec" << endl;
	return true;
}

bool cvtool::readPointCloudFromTxt_fast(const string filename, vector<Point3f> &pnts)
{

#if 0 //时间并没有上述方法快
	ifstream fi;
	string buffer;
	fi.open(filename);
	if (!fi.is_open())
	{
		cout << "读取文件错误！" << endl;
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
	cout << "读取文件耗时：" << t1 << " sec" << endl;
	return true;
#endif
	
	//获得文件句柄
	WCHAR wszClassName[256];
	memset(wszClassName, 0, sizeof(wszClassName));
	MultiByteToWideChar(CP_ACP, 0, filename.c_str(), strlen(filename.c_str()) + 1, wszClassName,
		sizeof(wszClassName) / sizeof(wszClassName[0]));

	HANDLE hFile = CreateFile(
		wszClassName,   //文件名
		GENERIC_READ | GENERIC_WRITE, //对文件进行读写操作
		FILE_SHARE_READ | FILE_SHARE_WRITE,
		NULL,
		OPEN_EXISTING,  //打开已存在文件
		FILE_ATTRIBUTE_NORMAL,
		0);

	//返回值size_high,size_low分别表示文件大小的高32位/低32位
	DWORD size_low, size_high;
	size_low = GetFileSize(hFile, &size_high);

	//创建文件的内存映射文件。   
	HANDLE hMapFile = CreateFileMapping(
		hFile,
		NULL,
		PAGE_READWRITE,  //对映射文件进行读写
		size_high,
		size_low,   //这两个参数共64位，所以支持的最大文件长度为16EB
		NULL);
	if (hMapFile == INVALID_HANDLE_VALUE)
	{
		std::cout << "Can't create file mapping.Error " << GetLastError << endl;
		CloseHandle(hFile);
		return false;
	}

	//把文件数据映射到进程的地址空间
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
	std::cout << "读取文件耗时：" << t1 << " sec" << endl;

	////撤销映射
	//UnmapViewOfFile(pvFile);
	////关闭文件
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
	std::cout << "保存文件耗时：" << t1 << " sec" << endl;
	return true;
}

bool cvtool::readfileTest(const string filename, vector<Point3f> &pnts)
{
	//方法1：一次读到内存
#if 1
	double t1 = cv::getTickCount();
	filebuf *pbuf;
	ifstream fileread;
	long size;
	char * buffer;
	// 要读入整个文件，必须采用二进制打开   
	fileread.open(filename, ios::binary);
	// 获取filestr对应buffer对象的指针（获得这个流对象的指针）
	pbuf = fileread.rdbuf();

	// 调用buffer对象方法获取文件大小  （当复位位置指针指向文件缓冲区的末尾时候pubseekoff返回的值就是整个文件流大小）
	size = pbuf->pubseekoff(0, ios::end, ios::in);
	pbuf->pubseekpos(0, ios::in);   //再让pbuf指向文件流开始位置

	// 分配内存空间  
	buffer = new char[size];

	// 获取文件内容  
	pbuf->sgetn(buffer, size); 

	fileread.close();
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	
	// 输出到标准输出  
	double t2 = cv::getTickCount();
	//cout.write(buffer, size);

//最佳方法按字符遍历整个buffer
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
				if (!isy)  //如果是x的值
				{
					if (!temp.empty())
					{
						isy = !isy;
						sscanf(temp.data(), "%f", &x);
						Pnts.x = x;
						temp = "";
					}
				}
				else                  //如果是y的值
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
		else   //这里是z
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
	//delete[] buffer;  //释放会出错，不释放没问题，估计是堆和栈的问题
	std::cout << "txt读入内存：" << t1 << " sec" << endl;
	std::cout << "txt转换数据：" << t2 << " sec" << endl;
	//整体字符分割
#if 0
	char *Surplus = strtok(buffer, " ");
	while (Surplus)
	{
		//判断当前截取片段有无回车，是就分割
		string str1 = Surplus;
		
		//string转char
		char str_char1[20];
		str1.copy(str_char1, 20, 0); //这里5，代表复制几个字符，0代表复制的位置
		//*(str_char1 + 20) = '/0';
		

		char key = '\r\n';  //指定一个字符
		char *strtemp;
		strtemp = strchr(str_char1, key);
		if (strtemp != NULL) //如果找到有回车
		{
			//去除回车
			char *enter = strtok(str_char1, "\r\n");
			//string转char
			string str2 = enter;
			char str_char2[20];
			str2.copy(str_char2, 20, 0); //这里5，代表复制几个字符，0代表复制的位置
			//*(str_char2 + 20) = '/0';

			double z = atof(str_char2);
			printf("%.5f\n", z);

			enter = strtok(NULL, "\r\n");
			//string转char
			string str3 = enter;
			char str_char3[20];
			str3.copy(str_char3, 20, 0); //这里5，代表复制几个字符，0代表复制的位置
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
	//方法2：一行行读取txt文件，时间太长了，平均一个点要0.00015s
#if 0
	double t1 = cv::getTickCount();
	string line;
	int n = 0;
	char *Surplus;
	ifstream fp(filename);//,ios::binary);
	while (!fp.eof())
	{
		double t2 = cv::getTickCount();
		//每个点所花时间
		getline(fp, line);


		//getlinr+stroke+vector<string>
#if 0
			vector<string> elems;
			char line_char[40];
			line.copy(line_char, 40, 0); //这里5，代表复制几个字符，0代表复制的位置
			char *s = strtok(line_char, " ");
			while (s != NULL) {
				elems.push_back(s);
				s = strtok(NULL, " ");
			}
			Point3f Pnts;
			Pnts.x = atof(elems[0]);


#endif

		//getline+strtok+string转char 
#if 0
		char line_char[40];
		line.copy(line_char, 40, 0); //这里5，代表复制几个字符，0代表复制的位置
		//*(line_char + 20) = '/0';   //字符串最后手动结尾，但是没什么影响
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
		cout << "当前点赋值时间： " << t2 << " sec，当前是第" <<n<<"个点"<< endl;
	}
	delete[] Surplus;
	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	cout << "读取txt时间： " << t1 << " sec" << endl;
#endif

	return true;
}

bool cvtool::writefileTest(const string filename, vector<Point3f> &pnts)
{
	//1.将点云团按x y z格式转换成一个整体字符串buffer
	double t1 = cv::getTickCount();
	//用ss直接读所有的数字，一起转成string（在读取函数时size大小出错）
	stringstream ss;
	for (int i = 0; i < pnts.size();i++)
	{
		ss << pnts[i].x << " " << pnts[i].y << " " << pnts[i].z << "\r\n";
	}
	string buffer(ss.str());
	//sprintf数字转字符串

	t1 = (cv::getTickCount() - t1) / cv::getTickFrequency();
	//2.将一个整体buffer一下写入二进制文件
	double t2 = cv::getTickCount();
	ofstream filewrite;
	filewrite.open(filename, ios::binary);
	filewrite << buffer;
	filewrite.close();
	t2 = (cv::getTickCount() - t2) / cv::getTickFrequency();
	std::cout << "txt转换数据：" << t1 << " sec" << endl;
	std::cout << "txt写入内存：" << t2 << " sec" << endl;
	return true;
}
