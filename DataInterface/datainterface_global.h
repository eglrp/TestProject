//*************************************************************************
//˵�����������ɸ�ƽ̨֮������ת���Ķ�̬��
//���ߣ�ZZL
//����ʱ�䣺2018/02/01
//��ע��
//*************************************************************************
#ifndef DATAINTERFACE_GLOBAL_H
#define DATAINTERFACE_GLOBAL_H

#include <QtCore/qglobal.h>

#ifdef DATAINTERFACE_LIB
# define DATAINTERFACE_EXPORT Q_DECL_EXPORT
#else
# define DATAINTERFACE_EXPORT Q_DECL_IMPORT
#endif

#endif // DATAINTERFACE_GLOBAL_H
