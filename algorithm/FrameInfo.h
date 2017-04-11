#ifndef	FRAMEINFO_H
#define FRAMEINFO_H
#include <qstring.h>
#include <qfile.h>
#include <qdatastream.h>
#include <iostream>
#include "definations.h"
class FrameInfoRecon
{
public:
	inline FrameInfoRecon(){};
	inline void save(QString filename)
	{
		QFile file(filename);
		if (file.open(QIODevice::WriteOnly))
		{
			QDataStream out(&file);
			out << *this;
		}
		file.close();
	}
	inline void load(QString filename)
	{
		QFile file(filename);
		if (file.open(QIODevice::ReadOnly))
		{
			QDataStream in(&file);
			in >> *this;
		}
		file.close();
	}
	friend QDataStream& operator<<(QDataStream& out, const FrameInfoRecon& info)
	{
		out << info.icp_suc;
		for(int i = 0; i < 4*4; i++)
			out<< info.rigid_transform[i];
		return out;
	}
	friend QDataStream& operator>>(QDataStream& in, FrameInfoRecon& info)
	{
		in >> info.icp_suc;
		for (int i = 0; i < 4 * 4; i++)
			in >> info.rigid_transform[i];
		return in;
	}
public:
	bool icp_suc = false;
	Tbx::Transfo rigid_transform = Tbx::Transfo::identity();
};


class FrameInfoDeform
{
public:
	inline FrameInfoDeform() {};
	inline void save(QString filename)
	{
		QFile file(filename);
		if (file.open(QIODevice::WriteOnly))
		{
			QDataStream out(&file);
			out << *this;
		}
		file.close();
	}
	inline void load(QString filename)
	{
		QFile file(filename);
		if (file.open(QIODevice::ReadOnly))
		{
			QDataStream in(&file);
			in >> *this;
		}
		file.close();
	}
	friend QDataStream& operator<<(QDataStream& out, const FrameInfoDeform& info)
	{
		out << info.key_frame_index << info.m_model_state;
		return out;
	}
	friend QDataStream& operator >> (QDataStream& in, FrameInfoDeform& info)
	{
		in >> info.key_frame_index >> info.m_model_state;
		return in;
	}
public:
	int key_frame_index = -1;
	int key_frame_subindex = -1;
	int m_model_state;
};

#endif
