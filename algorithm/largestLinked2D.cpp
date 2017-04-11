#include "largestLinked2D.h"
#include <queue>
#include <map>

void setImageColor_(std::vector<dfusion::PixelRGBA> &image,
	const std::set<PixelPos> &posSet, dfusion::PixelRGBA RGBA, float w = 1)
{
	std::set<PixelPos>::iterator iter;
	for (iter = posSet.begin(); iter != posSet.end(); iter++)
	{
		if (w < 0) w = 0;
		else if (w > 1) w = 1;
		dfusion::PixelRGBA color;
		color.r = w*RGBA.r + (1 - w)*image[*iter].r;
		color.g = w*RGBA.g + (1 - w)*image[*iter].g;
		color.b = w*RGBA.b + (1 - w)*image[*iter].b;
		color.a = 255;
		image[*iter] = color;

	}
}
void setImageColor_(std::vector<dfusion::PixelRGBA> &image,
	const std::vector<PixelPos> &posList, dfusion::PixelRGBA RGBA, float w = 1)
{
	
	for (int i = 0; i < posList.size(); i++)
	{
		int index = posList[i];
		if (w < 0) w = 0;
		else if (w > 1) w = 1;
		dfusion::PixelRGBA color;
		color.r = w*RGBA.r + (1 - w)*image[index].r;
		color.g = w*RGBA.g + (1 - w)*image[index].g;
		color.b = w*RGBA.b + (1 - w)*image[index].b;
		color.a = 255;
		image[index] = color;

	}
}
void savePixelRGBA_(QString name, const std::vector<dfusion::PixelRGBA>* color)
{
	QImage img(dfusion::KINECT_WIDTH,
		dfusion::KINECT_HEIGHT, QImage::Format_RGBA8888);
	const dfusion::PixelRGBA* src = color->data();
	for (int y = 0; y < dfusion::KINECT_HEIGHT; y++)
		for (int x = 0; x < dfusion::KINECT_WIDTH; x++)
		{
			dfusion::PixelRGBA p = src[y*dfusion::KINECT_WIDTH + x];
			img.setPixel(x, y, qRgba(p.r, p.g, p.b, p.a));
		}
	printf("%s\n", name.toStdString().c_str());
	img.save(name);
}

void getLinked2D(BooleanMap & bmap, int row, int col, std::set<PixelPos>& linkedSet)
{
	
	if (!bmap.inMap(row, col))
	{
		linkedSet.clear();
		return;
	}

	std::queue<PixelPos> que;
	if (bmap.inMap(row, col) && bmap.value(row, col))
		que.push(bmap.index1D(row, col));
	//printf("row = %d, col = %d\n", row, col);
	PixelPos ppos;
	ppos = bmap.index1D(row, col);
	bmap.setValue(ppos, false);
	int _row, _col;
	int _row_to_add, _col_to_add;
	bool val;
	linkedSet.clear();
	/*int row_offset[8] = {-1, 0, 1, 0, -2, 0, 2, 0};
	int col_offset[8] = {0, 1, 0,-1, 0, -2, 0, 2};*/
	int row_offset[8] = { -1, 0, 1, 0};
	int col_offset[8] = { 0, 1, 0,-1};
	while (!que.empty())
	{
		//printf("que size = %d\n", que.size());
		ppos = que.front();
		que.pop();
		linkedSet.insert(ppos);
		bmap.index2D(ppos, _row, _col);
		//bmap.setValue(ppos, false);
		//printf("_row = %d, _col = %d\n", _row, _col);
		
		for (int i = 0; i < 8; i++)
		{
			_row_to_add = _row + row_offset[i];
			_col_to_add = _col + col_offset[i];
			//printf("_row_to_add = %d, _col_to_add = %d\n", _row_to_add, _col_to_add);
			ppos = bmap.index1D(_row_to_add, _col_to_add);
			if (bmap.inMap(_row_to_add, _col_to_add) &&
				bmap.value(_row_to_add, _col_to_add) &&
				linkedSet.find(ppos) == linkedSet.end())
			{
				
				//printf("push (%d,%d) into linked\n", _row_to_add, _col_to_add);
				que.push(ppos);
				bmap.setValue(ppos, false);
			}
		}
				
			
		//getchar();
	}
	//fclose(fp);
	//system("pause");
	//getchar();
}

void bildTestSample(BooleanMap &bmap, std::set<PixelPos>& ppSet)
{
	int row, col;
	int w,h;
	int _row, _col;
	int row_offset, col_offset;

	ppSet.clear();


	row = 10;
	col = 10;
	w = h = 20;
	for (row_offset = 0; row_offset < h; row_offset++)
	{
		for (col_offset = 0; col_offset < w; col_offset++)
		{
			_row = row + row_offset;
			_col = col + col_offset;

			int index = bmap.index1D(_row, _col);
			ppSet.insert(index);
		}
	}

	row = 100;
	col = 100;
	w = h = 40;
	for (row_offset = 0; row_offset < h; row_offset++)
	{
		for (col_offset = 0; col_offset < w; col_offset++)
		{
			_row = row + row_offset;
			_col = col + col_offset;

			int index = bmap.index1D(_row, _col);
			ppSet.insert(index);
		}
	}
}


void getLargestLinked2D(int width, int height,std::set<PixelPos>& input, std::set<PixelPos>& output,
	const std::vector<dfusion::depthtype> &depthMap, float depth_range, const std::vector<dfusion::PixelRGBA> *colromap)
{

	//printf("input size = %d\n", input.size());
	BooleanMap bmap(width, height);
	//printf("bmap built\n");
	//bildTestSample(bmap, input);
	std::vector<std::set<PixelPos>> linkedSets;
	std::vector<bool> checked;
	std::vector<PixelPos> trueList;
	std::set<PixelPos>::iterator iter;
	float depth;
	int row, col;
	int index;
	float min_dep = FLT_MAX;
	if (input.size() == 0)
		return;
	for (iter = input.begin(); iter != input.end(); iter++)
	{
		index = (int)*iter;
		//printf("index = %d\n", index);
		depth = depthMap[index];
		if(depth == 0)
			continue;
		//printf("depth = %f\n", depth);
		if (depth < min_dep)
			min_dep = depth;
	}
	float minDepth = min_dep;
	float maxDetph = min_dep + depth_range;
	
	
	for (iter = input.begin(); iter != input.end(); iter++)
	{
		index = *iter;
		depth = depthMap[index];
		//printf("depth = %f\n", depth);
		if (depth >= minDepth && depth <= maxDetph)
		{
			trueList.push_back(index);
		}
	}
	bmap.setValueByList(trueList, true);
	if (colromap)
	{
		dfusion::PixelRGBA rgba;
		std::vector<dfusion::PixelRGBA> cmap = *colromap;
		rgba.r = 0;
		rgba.g = 0;
		rgba.b = 255;
		rgba.a = 255;
		setImageColor_(cmap, trueList, rgba);
		savePixelRGBA_("handcover_filtered.png", &cmap);
	}
	int max_index = 0, max_size = 0;
	for (int i = 0; i < trueList.size(); i++)
	{
		//printf("i = %d\n", i);
		index = trueList[i];
		if(!bmap.value(index))
			continue;
		//printf("i = %d\n", i);
		std::set<PixelPos> linked;
		linked.clear();
		
		bmap.index2D(index, row, col);
		//printf("row = %d, col = %d\n", row, col);
		getLinked2D(bmap, row, col, linked);
		if (linked.size() > max_size)
		{
			
			max_size = linked.size();
			max_index = linkedSets.size();
		}
		linkedSets.push_back(linked);
		bmap.setValueBySet(linked, false);
	}

	output.clear();
	for (iter = linkedSets[max_index].begin(); iter != linkedSets[max_index].end(); iter++)
	{
		output.insert(*iter);
	}
	if (colromap)
	{
		dfusion::PixelRGBA rgba;
		std::vector<dfusion::PixelRGBA> cmap = *colromap;
		rgba.r = 0;
		rgba.g = 0;
		rgba.b = 255;
		rgba.a = 255;
		setImageColor_(cmap, output, rgba);
		savePixelRGBA_("handcover_largestLinked.png", &cmap);
	}
	
}
ldp::Float3 getCenter(int width, int height,const std::vector<dfusion::depthtype> &depthMap, dfusion::Intr intr,
	const std::set<PixelPos>& pixelSet)
{
	ldp::Float3 sum = ldp::Float3(0, 0, 0);
	ldp::Float3 p_F;
	float3 p;
	//std::map<int, ldp::Float3> vmap;
	int row, col, index;
	float depth;
	std::set<PixelPos>::iterator iter;
	for (iter = pixelSet.begin(); iter != pixelSet.end(); iter++)
	{
		index = *iter;
		row = index / width;
		col = index % width;
		depth = depthMap[index];

		p = intr.uvd2xyz(col, row, depth);
		p_F = ldp::Float3(p.x, p.y, p.z);

		sum += p_F;
		
	}
	return sum / pixelSet.size();
}
ldp::Float3 getBBCenter(int width, int height, const std::vector<dfusion::depthtype> &depthMap, dfusion::Intr intr,
	const std::set<PixelPos>& pixelSet)
{
	ldp::Float3 sum = ldp::Float3(0, 0, 0);
	ldp::Float3 p_F;
	float3 p;
	//std::map<int, ldp::Float3> vmap;
	int row, col, index;
	float depth;
	std::set<PixelPos>::iterator iter;
	ldp::Float3 min_b = ldp::Float3(FLT_MAX, FLT_MAX, FLT_MAX);
	ldp::Float3 max_b = ldp::Float3(0, 0, 0);
	for (iter = pixelSet.begin(); iter != pixelSet.end(); iter++)
	{
		index = *iter;
		row = index / width;
		col = index % width;
		depth = depthMap[index];

		p = intr.uvd2xyz(col, row, depth);
		p_F = ldp::Float3(p.x, p.y, p.z);
		
		for (int i = 0; i < 3; i++)
		{
			if (p_F[i] > max_b[i])
			{
				max_b[i] = p_F[i];
			}
			if (p_F[i] < min_b[i])
			{
				min_b[i] = p_F[i];
			}
		}
	}

	return (min_b + max_b)/2;
}
ldp::Float3 getObjectPixel(int width, int height, const std::set<PixelPos>& handPixel, const std::set<PixelPos>& largest_linked_handPixel,
	std::set<PixelPos> &objectPixel, const std::vector<dfusion::depthtype> &depthMap,
	dfusion::Intr intr, ldp::Float3 bbox[], const std::vector<dfusion::PixelRGBA> *colromap)
{
	ldp::Float3 handCenter = getCenter(width, height, depthMap, intr, largest_linked_handPixel);
	
	//printf("hand center = (%f, %f, %f)\n", handCenter.x(), handCenter.y(), handCenter.z());
	ldp::Float3 sum = ldp::Float3(0, 0, 0);
	ldp::Float3 p_F;
	float3 p;
	int row, col, index;
	float depth;
	std::set<PixelPos>::iterator iter;
	
	float dis_x = bbox[1].x() - bbox[0].x();
	float dis_y = bbox[1].y() - bbox[0].y();
	float dis_z = bbox[1].z() - bbox[0].z();
	float maxDis = max(max(dis_x, dis_y), dis_z)*1.5;
	//printf("maxDis = %f\n", maxDis);
	objectPixel.clear();
	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			index = row*width + col;
			depth = depthMap[index];
			p = intr.uvd2xyz(col, row, depth);
			p_F = ldp::Float3(p.x, p.y, p.z);
			
			float dis = sqrt(handCenter.sqrDist(p_F));
			if (handPixel.find(index) == handPixel.end() && sqrt(handCenter.sqrDist(p_F)) < maxDis * 1000)
			//if (dis < maxDis * 1000)
			{
				
				objectPixel.insert(index);
			}
		}
	}
	if (colromap)
	{
		dfusion::PixelRGBA rgba;
		std::vector<dfusion::PixelRGBA> cmap = *colromap;
		rgba.r = 255;
		rgba.g = 0;
		rgba.b = 0;
		rgba.a = 255;
		setImageColor_(cmap, objectPixel, rgba);
		savePixelRGBA_("object.png", &cmap);
	}
	//printf("getObjectPixel:objectPixel size = %d\n", objectPixel.size());
	getLargestLinked2D(width, height, objectPixel, objectPixel, depthMap, dis_z * 1.2 * 1000);
	if (colromap)
	{
		dfusion::PixelRGBA rgba;
		std::vector<dfusion::PixelRGBA> cmap = *colromap;
		rgba.r = 255;
		rgba.g = 0;
		rgba.b = 0;
		rgba.a = 255;
		setImageColor_(cmap, objectPixel, rgba);
		savePixelRGBA_("object_largestLinked.png", &cmap);
	}
	return getBBCenter(width, height, depthMap, intr, objectPixel);
}
