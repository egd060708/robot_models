/*! @file	dataDisplay.h
 *  @brief	webots绘图驱动器
 *	@author	zzr
 *  @date	2023.9.21
 *
 *	调用API详见webots官方文档
 */
#pragma once
#include "myMatrices.h"
#include <webots/Display.hpp>
#include "Upper_Public.h"
using namespace webots;
USING_NAMESPACE_MM

#define DATAMAX 10.f	// 最大量程
#define ANXISDIVID 3	// 坐标轴划分个数
#define ANXISCOLOR getcolor(100,100,100)	// 坐标轴颜色

/* 定义模板类：视窗宽度，高度 */
template<uint8_t channelNum>
class dataDisplay {
private:
	myMatrices<int>* anxis;			// 坐标系矩阵
	myMatrices<int>* currentDisp;	// 当前要画的矩阵
	myMatrices<int>* lastDisp;		// 上次画的矩阵
	myMatrices<uint8_t>* isUpdate;	// 是否需要更新绘图
	myMatrices<int>* transfer;		// 转移矩阵
	Display* tag;

	uint16_t width = 0;
	uint16_t height = 0;

	/* 设置对应通道颜色 */
	int getcolor(int r, int g, int b)
	{
		return (r << 16 | g << 8 | b);
	}

	/* 数值和纵坐标（行数）转换 */
	int num2row(float _num)
	{
		float temp = _num / DATAMAX * height / 2.f;// 算出该数据点在纵坐标的位置
		int std_temp = (int)temp;
		if (temp - std_temp > 0.5)
		{
			std_temp += 1;// 四舍五入计算坐标
		}
		int fact_temp = height / 2 - std_temp - 1;//对应像素点的位置
		return upper::constrain(fact_temp, 0, height - 1);
	}

	/* 更新绘制数据 */
	void currentUpdate(float _data[channelNum])
	{
		*currentDisp = (*currentDisp) * (*transfer);// 矩阵左移一列
		for (int i = 0; i < channelNum; i++)
		{
			int _color;
			switch (i)
			{
			case 0: _color = getcolor(255, 0, 0); break;// 红
			case 1: _color = getcolor(0, 255, 0); break;// 绿
			case 2: _color = getcolor(0, 0, 255); break;// 蓝
			case 3: _color = getcolor(0, 255, 255); break;// 青
			case 4: _color = getcolor(255, 0, 255); break;// 紫
			case 5: _color = getcolor(255, 255, 0); break;// 黄
			default:break;
			}
			currentDisp->setElement(num2row(_data[i]), width - 1, _color);// 把更新的颜色写入最后一列向量里
		}

		// 将坐标系插入新坐标中
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				int dataPixel = currentDisp->getElement(i, j);
				int anxisPixel = anxis->getElement(i, j);
				if (dataPixel == 0 && anxisPixel == ANXISCOLOR)
				{
					currentDisp->setElement(i, j, ANXISCOLOR);
				}
			}
		}
	}

	/* 判断要重新绘制的点 */
	void drawingJudge()
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				if (currentDisp->getElement(i, j) != lastDisp->getElement(i, j))
				{
					isUpdate->setElement(i, j, 1);// 对需要更改的像素点赋值1
				}
				else
				{
					isUpdate->setElement(i, j, 0);
				}
			}
		}
	}

public:
	/* 构造函数 */
	dataDisplay(Display* _tag):tag(_tag){
		static_assert((channelNum > 0) && (channelNum <= 6),
			"display channel number should be in [1,6]");// 断言通道数量是否在范围内

		width = tag->getWidth();	// 获取矩阵宽度
		height = tag->getHeight();	// 获取矩阵高度

		currentDisp = new myMatrices<int>(height, width);
		lastDisp = new myMatrices<int>(height, width);
		isUpdate = new myMatrices<uint8_t>(height, width);
		transfer = new myMatrices<int>(width);
		anxis = new myMatrices<int>(height, width);

		for (int i = 0; i < width -1; i++)
		{
			transfer->setElement(i + 1, i,1); // 把相应位置赋1
		}

		// 填入坐标系内容
		int anxis_color = ANXISCOLOR;
		for (int i = 0; i < ANXISDIVID; i++)
		{
			for (int j = 0; j < width - 1; j++)
			{
				anxis->setElement(num2row(i * DATAMAX / (float)ANXISDIVID), j, anxis_color);
				anxis->setElement(num2row(-i * DATAMAX / (float)ANXISDIVID), j, anxis_color);
			}
		}
	}

	/* 曲线绘制 */
	void drawPixel()
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				if (isUpdate->getElement(i, j) == 1)
				{
					tag->setColor(currentDisp->getElement(i,j));// 取出对应颜色
					tag->drawPixel(j, i);// 画点
				}
			}
		}
	}

	/* 调用函数 */
	void sendCtrl(float _data[channelNum])
	{
		currentUpdate(_data);
		drawingJudge();
		drawPixel();
		*lastDisp = *currentDisp;// 保存当前矩阵状态
	}

};