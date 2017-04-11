#pragma once
#include <set>
#include <vector>
#include "definations.h"
#include "ldp_basic_vec.h"
#include <QImage>
typedef struct BooleanMap
{
	std::vector<bool> map;
	std::vector<int> t;
	int width;
	int height;

	BooleanMap(int width, int height)
	{
		this->width = width;
		this->height = height;
		map.resize(width*height, false);
	}
	BooleanMap(int width, int height, const std::set<PixelPos>& trueSet)
	{
		this->width = width;
		this->height = height;
		map.resize(width*height, false);
		setValueBySet(trueSet, true);
	}
	bool inMap(int row, int col)
	{
		if (row < height && col < width && row >= 0 && col >= 0)
			return true;
		else
			return false;
	}
	bool value(int index) { return map[index]; }
	bool value(int row, int col) { return map[index1D(row, col)]; }
	int index1D(int row, int col) {return row*width + col;}
	void index2D(int index, int &row, int &col)
	{
		row = index / width;
		col = index % width;
	}
	void setValue(int index, bool val)
	{
		map[index] = val;
	}
	void setValue(int row, int col, bool val)
	{
		if (inMap(row,col))
		{
			printf("BooleanMap::setValue: wrong index\n");
			return;
		}
		int index = index1D(row, col);
		setValue(index, val);
	}
	void setValueBySet(const std::set<PixelPos>& posSet, bool val)
	{
		std::set<PixelPos>::iterator iter;
		for (iter = posSet.begin(); iter != posSet.end(); iter++)
		{
			setValue((int)*iter, val);
		}
	}
	void setValueByList(std::vector<PixelPos>& posList, bool val)
	{
		std::vector<PixelPos>::iterator iter;
		for (iter = posList.begin(); iter != posList.end(); iter++)
		{
			setValue((int)*iter, val);
		}
	}
	void save(QString name)
	{
		QImage img(dfusion::KINECT_WIDTH,
			dfusion::KINECT_HEIGHT, QImage::Format_RGBA8888);
		for (int row = 0; row < height; row++)
		{
			for (int col = 0; col < width; col++)
			{
				if(value(row, col))
					img.setPixel(col, row, qRgba(255, 0, 0, 255));
			}
		}
		img.save(name);
	}

}BooleanMap;
void getLinked2D(BooleanMap & bmap, int row, int col, std::set<PixelPos> &linkedSet);
void getLargestLinked2D(int width, int height, std::set<PixelPos>& input, 
	std::set<PixelPos>& output, const std::vector<dfusion::depthtype> &depthMap, 
	float depth_range = 100, const std::vector<dfusion::PixelRGBA> *colromap = NULL);
ldp::Float3 getObjectPixel(int width, int height, const std::set<PixelPos>& handPixel, const std::set<PixelPos>& largest_linked_handPixel,
	std::set<PixelPos> &objectPixel, const std::vector<dfusion::depthtype> &depthMap,
	dfusion::Intr intr, ldp::Float3 bbox[], const std::vector<dfusion::PixelRGBA> *colromap = NULL);
//void  getLargestLinked2DWithDepthFilter(const std::set<PixelPos> &input, std::set<PixelPos> &output, std::vector<dfusion::depthtype> &depthMap, );
