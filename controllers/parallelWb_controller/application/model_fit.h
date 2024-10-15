/*! @file	model_fit.h
 *  @brief	模型矩阵生成类型
 *	@author	zzr
 *  @date	2023.9.12
 *
 *  1、setFunctions
 *	2、modelGenerate
 */
#ifndef _MODEL_FIT_H_
#define _MODEL_FIT_H_

#include <qpOASES.hpp>
#include "myMatrices.h"

USING_NAMESPACE_MM
USING_NAMESPACE_QPOASES

/* 模板参数：矩阵模型的行数，矩阵模型的列数，拟合函数的阶数 */
template<uint_t rows,uint_t cols,uint_t order>
class modelFit {
private:
	MATRIX model = MATRIX(rows * cols, order + 1);//行数区分阶次参数，列数区分矩阵位置参数

	/* 递归调用求解方程结果: 自变量，方程系数，方程阶数 */
	real_t functionSolve(const real_t _x, const real_t _para[order + 1], uint_t _orderNum)
	{
		if (_orderNum)
		{
			return _para[_orderNum] * powf(_x, _orderNum) + functionSolve(_x, _para, _orderNum - 1);
		}
		else
		{
			return _para[_orderNum];
		}
	}
public:
	/* 模型生成函数	形参：自变量 */
	MATRIX modelGenerate(real_t _x) {
		MATRIX result(rows, cols);
		for(int i=0;i < rows;i++)
			for (int j = 0; j < cols; j++) {
				real_t tmp[order + 1];
				model.getRowArray(tmp, i * cols + j);
				result.setElement(i, j, functionSolve(_x,tmp, order));
			}
		return result;
	}
	/* 设置拟合方程参数，自动按照方程个数进行循环写入 */
	void setFunctions(const real_t _functions[rows*cols*(order + 1)])
	{
		model.setArray(_functions, rows * cols * (order + 1));
	}

};

#endif
